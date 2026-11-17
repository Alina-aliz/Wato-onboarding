#pragma once
#include "navigation_common/types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
namespace robot {
inline double yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
}
inline Grid fromMessage(const nav_msgs::msg::OccupancyGrid& m) {
  Grid g;
  g.width = static_cast<int>(m.info.width); g.height = static_cast<int>(m.info.height);
  g.resolution = m.info.resolution;
  g.origin_x = m.info.origin.position.x; g.origin_y = m.info.origin.position.y;
  g.data = m.data; return g;
}
inline nav_msgs::msg::OccupancyGrid toMessage(const Grid& g) {
  nav_msgs::msg::OccupancyGrid m;
  m.info.width = g.width; m.info.height = g.height; m.info.resolution = g.resolution;
  m.info.origin.position.x = g.origin_x; m.info.origin.position.y = g.origin_y;
  m.info.origin.orientation.w = 1.0; m.data = g.data; return m;
}
}
