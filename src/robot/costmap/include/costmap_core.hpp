#pragma once
#include "navigation_common/types.hpp"
namespace robot {
struct Scan {
  double angle_min = 0, angle_increment = 0, range_min = 0, range_max = 0;
  std::vector<float> ranges;
};
struct CostmapConfig {
  double resolution = 0.1, size = 42.0, robot_radius = 1.85, inflation_radius = 2.5;
};
class CostmapCore {
 public:
  explicit CostmapCore(CostmapConfig config = {});
  Grid build(const Scan& scan) const;
 private:
  CostmapConfig config_;
};
}
