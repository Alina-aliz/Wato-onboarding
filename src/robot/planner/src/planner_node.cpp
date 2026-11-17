#include "planner_node.hpp"
#include "navigation_common/ros.hpp"
using Clock=std::chrono::steady_clock;
PlannerNode::PlannerNode():Node("planner_node"),core_(robot::PlannerConfig{
  declare_parameter("obstacle_threshold",99),declare_parameter("unknown_cost",3.0),
  declare_parameter("cost_weight",4.0)}) {
  frame_=declare_parameter<std::string>("global_frame","sim_world");
  offset_=declare_parameter("lidar_to_axle",1.3);
  tolerance_=declare_parameter("goal_tolerance",0.25);timeout_=declare_parameter("goal_timeout",180.0);
  path_pub_=create_publisher<nav_msgs::msg::Path>("/path",1);
  map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      if(m->header.frame_id!=frame_ || std::abs(robot::yaw(m->info.origin.orientation))>1e-6) return;
      grid_=robot::fromMessage(*m);map_received_=Clock::now();
    });
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(m->header.frame_id!=frame_) return;
      odom_=m;odom_received_=Clock::now();
    });
  goal_sub_=create_subscription<geometry_msgs::msg::PointStamped>("/goal_point",10,
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m){
      // An empty frame is accepted as global for simple command-line goals.
      if((!m->header.frame_id.empty() && m->header.frame_id!=frame_) ||
         !std::isfinite(m->point.x)||!std::isfinite(m->point.y)) {
        RCLCPP_WARN(get_logger(),"Goal rejected: expected finite coordinates in %s",frame_.c_str());return;
      }
      goal_={m->point.x,m->point.y};goal_time_=progress_time_=Clock::now();
      progress_position_={INFINITY,INFINITY};state_=State::NAVIGATING;
      current_path_.clear();
      preferred_path_.clear();
      publish({}); // Invalidate the previous goal's path immediately.
      tick();
    });
  timer_=create_wall_timer(std::chrono::milliseconds(500),std::bind(&PlannerNode::tick,this));
}
void PlannerNode::publish(const std::vector<robot::Point>& points) {
  nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
  for(size_t i=0;i<points.size();++i) {
    geometry_msgs::msg::PoseStamped p;p.header=path.header;
    p.pose.position.x=points[i].x;p.pose.position.y=points[i].y;
    double a=0;
    if(i+1<points.size()) a=std::atan2(points[i+1].y-points[i].y,points[i+1].x-points[i].x);
    p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);
    path.poses.push_back(p);
  }
  path_pub_->publish(path);
}
void PlannerNode::tick() {
  if(state_==State::WAITING_FOR_GOAL) {publish({});return;}
  const auto time=Clock::now();
  auto age=[&](Clock::time_point t){return std::chrono::duration<double>(time-t).count();};
  if(!odom_ || age(odom_received_)>0.6 || age(map_received_)>1.5 || !grid_.valid()) {publish({});return;}
  const auto& pose=odom_->pose.pose;
  double a=robot::yaw(pose.orientation);
  robot::Point axle{pose.position.x-offset_*std::cos(a),pose.position.y-offset_*std::sin(a)};
  if(robot::distance(axle,goal_)<=tolerance_) {
    state_=State::WAITING_FOR_GOAL;publish({});RCLCPP_INFO(get_logger(),"Goal reached");return;
  }
  if(robot::distance(axle,progress_position_)>0.15) {progress_position_=axle;progress_time_=time;}
  if(age(goal_time_)>timeout_ || age(progress_time_)>30) {
    state_=State::WAITING_FOR_GOAL;publish({});
    RCLCPP_WARN(get_logger(),"Navigation timed out; stopped. Send another goal to retry.");return;
  }
  if(!core_.pathUsable(grid_,current_path_,axle)) {
    current_path_=core_.plan(grid_,axle,goal_,preferred_path_);
    if(!current_path_.empty()) {
      preferred_path_=current_path_;
      RCLCPP_INFO(get_logger(),"New route: %zu waypoints",current_path_.size());
    }
  }
  if(current_path_.empty()) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
    "No valid path: retrying. Check map bounds and obstacle clearance.");
  publish(current_path_); // Refresh the watchdog without changing a valid route.
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PlannerNode>());rclcpp::shutdown();return 0;
}
