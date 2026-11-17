#include "control_node.hpp"
#include "navigation_common/ros.hpp"
using Clock=std::chrono::steady_clock;
ControlNode::ControlNode():Node("control_node"),core_(robot::ControlConfig{
  declare_parameter("lookahead",0.5),declare_parameter("goal_tolerance",0.25),
  declare_parameter("max_speed",0.4),declare_parameter("max_angular",0.6),
  declare_parameter("rotate_threshold",0.6)}) {
  frame_=declare_parameter<std::string>("global_frame","sim_world");
  offset_=declare_parameter("lidar_to_axle",1.3);radius_=declare_parameter("robot_radius",1.85);
  velocity_pub_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
  path_sub_=create_subscription<nav_msgs::msg::Path>("/path",1,
    [this](nav_msgs::msg::Path::ConstSharedPtr m){
      path_.clear();path_time_=Clock::now();
      if(m->header.frame_id!=frame_) return;
      for(const auto& p:m->poses) path_.push_back({p.pose.position.x,p.pose.position.y});
    });
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(m->header.frame_id==frame_) {odom_=m;odom_time_=Clock::now();}
    });
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/lidar",rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_=m;scan_time_=Clock::now();});
  timer_=create_wall_timer(std::chrono::milliseconds(50),std::bind(&ControlNode::tick,this));
}
bool ControlNode::collisionRisk(robot::Velocity velocity) const {
  if(!scan_ || scan_->ranges.empty() || !std::isfinite(scan_->angle_min) ||
     !std::isfinite(scan_->angle_increment)) return true;
  bool valid=false;
  for(size_t i=0;i<scan_->ranges.size();++i) {
    double r=scan_->ranges[i];
    if(std::isnan(r)||r<scan_->range_min||r==-INFINITY) continue;
    valid=true;
    if(!std::isfinite(r)||r>=scan_->range_max) continue;
    double a=scan_->angle_min+i*scan_->angle_increment;
    robot::Point hit{offset_+r*std::cos(a),r*std::sin(a)};
    // A conservative circular footprint also covers in-place turns. Test a
    // short braking horizon along the commanded differential-drive arc.
    for(int step=0;step<=10;++step) {
      double t=step*0.1,x=velocity.linear*t,y=0;
      if(std::abs(velocity.angular)>1e-6) {
        x=velocity.linear*std::sin(velocity.angular*t)/velocity.angular;
        y=velocity.linear*(1-std::cos(velocity.angular*t))/velocity.angular;
      }
      if(robot::distance(hit,{x,y})<radius_+0.05) return true;
    }
  }
  return !valid;
}
void ControlNode::tick() {
  geometry_msgs::msg::Twist message;const auto time=Clock::now();
  auto age=[&](Clock::time_point t){return std::chrono::duration<double>(time-t).count();};
  if(odom_ && age(odom_time_)<0.6 && age(path_time_)<1.5 && age(scan_time_)<0.6) {
    const auto& p=odom_->pose.pose;double a=robot::yaw(p.orientation);
    robot::Pose axle{p.position.x-offset_*std::cos(a),p.position.y-offset_*std::sin(a),a};
    auto command=core_.command(path_,axle);
    if(!collisionRisk(command)) {message.linear.x=command.linear;message.angular.z=command.angular;}
  }
  velocity_pub_->publish(message); // Publish zero on every missing/stale input.
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<ControlNode>();
  rclcpp::spin(node);rclcpp::shutdown();return 0;
}
