#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "planner_core.hpp"
#include <chrono>
class PlannerNode : public rclcpp::Node {
 public: PlannerNode();
 private:
  enum class State { WAITING_FOR_GOAL, NAVIGATING };
  void tick();
  void publish(const std::vector<robot::Point>& points);
  robot::PlannerCore core_;
  State state_=State::WAITING_FOR_GOAL;
  robot::Grid grid_;
  robot::Point goal_,progress_position_;
  std::vector<robot::Point> current_path_;
  std::vector<robot::Point> preferred_path_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::string frame_;
  double offset_,tolerance_,timeout_;
  std::chrono::steady_clock::time_point odom_received_,map_received_,goal_time_,progress_time_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
