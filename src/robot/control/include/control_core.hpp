#pragma once
#include "navigation_common/types.hpp"
namespace robot {
struct ControlConfig {
  double lookahead=0.5, goal_tolerance=0.25, max_speed=0.4, max_angular=0.6, rotate_threshold=0.6;
};
class ControlCore {
 public:
  explicit ControlCore(ControlConfig config = {});
  Velocity command(const std::vector<Point>& path,Pose axle) const;
 private:
  ControlConfig config_;
};
}
