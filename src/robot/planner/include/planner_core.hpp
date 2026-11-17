#pragma once
#include "navigation_common/types.hpp"
namespace robot {
struct PlannerConfig { int obstacle_threshold=99; double unknown_cost=3.0, cost_weight=4.0; };
class PlannerCore {
 public:
  explicit PlannerCore(PlannerConfig config = {});
  std::vector<Point> plan(const Grid& grid,Point start,Point goal,
                          const std::vector<Point>& preferred_path = {}) const;
  bool segmentClear(const Grid& grid,Point a,Point b) const;
  bool pathUsable(const Grid& grid,const std::vector<Point>& path,Point position) const;
 private:
  bool traversable(const Grid& grid,int x,int y) const;
  PlannerConfig config_;
};
}
