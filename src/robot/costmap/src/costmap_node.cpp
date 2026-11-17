#include "costmap_node.hpp"
#include "navigation_common/ros.hpp"
CostmapNode::CostmapNode():Node("costmap_node"),core_(robot::CostmapConfig{
  declare_parameter("resolution",0.1),declare_parameter("size",42.0),
  declare_parameter("robot_radius",1.85),declare_parameter("inflation_radius",2.5)}) {
  map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",1);
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/lidar",rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::scanCallback,this,std::placeholders::_1));
}
void CostmapNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
  if(scan->header.frame_id.empty() || scan->ranges.empty()) return;
  robot::Scan data{scan->angle_min,scan->angle_increment,scan->range_min,scan->range_max,scan->ranges};
  auto map=robot::toMessage(core_.build(data));
  map.header=scan->header; map_pub_->publish(map);
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<CostmapNode>());rclcpp::shutdown();return 0;
}
