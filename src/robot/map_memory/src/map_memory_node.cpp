#include "map_memory_node.hpp"
#include "navigation_common/ros.hpp"
#include "tf2/exceptions.h"
#include <chrono>
MapMemoryNode::MapMemoryNode():Node("map_memory_node"),core_(
  declare_parameter("cells",300),declare_parameter("resolution",0.2)) {
  frame_=declare_parameter<std::string>("global_frame","sim_world");
  tf_=std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
  map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());
  costmap_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap",1,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){local_=m;pending_=true;});
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m){odom_=m;});
  timer_=create_wall_timer(std::chrono::milliseconds(250),std::bind(&MapMemoryNode::update,this));
  update(); // Publish an unknown map immediately, including while stationary.
}
void MapMemoryNode::update() {
  if(pending_ && local_) {
    robot::Pose pose;bool transformed=false;
    try {
      // Use the scan timestamp, not the current robot pose: this avoids smearing
      // walls while turning. No blocking TF wait inside the single-thread executor.
      auto t=tf_->lookupTransform(frame_,local_->header.frame_id,rclcpp::Time(local_->header.stamp));
      pose={t.transform.translation.x,t.transform.translation.y,robot::yaw(t.transform.rotation)};
      transformed=true;
    } catch(const tf2::TransformException&) {
      // The provided odometry is the lidar pose. Use it only for the same frame
      // and a close timestamp; otherwise leave this scan pending until TF arrives.
      if(odom_ && odom_->header.frame_id==frame_ && odom_->child_frame_id==local_->header.frame_id &&
         std::abs((rclcpp::Time(odom_->header.stamp)-rclcpp::Time(local_->header.stamp)).seconds())<0.1) {
        const auto& p=odom_->pose.pose;
        pose={p.position.x,p.position.y,robot::yaw(p.orientation)};transformed=true;
      }
    }
    if(transformed) {
      if(std::abs(robot::yaw(local_->info.origin.orientation))<1e-6)
        core_.integrate(robot::fromMessage(*local_),pose);
      pending_=false;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Waiting for scan transform into %s",frame_.c_str());
    }
  }
  auto map=robot::toMessage(core_.grid());map.header.frame_id=frame_;map.header.stamp=now();
  map_pub_->publish(map);
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<MapMemoryNode>());rclcpp::shutdown();return 0;
}
