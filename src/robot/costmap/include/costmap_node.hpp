#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
class CostmapNode : public rclcpp::Node {
 public: CostmapNode();
 private:
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  robot::CostmapCore core_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};
