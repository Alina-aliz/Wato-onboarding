#include "costmap_core.hpp"
#include <utility>
namespace robot {
CostmapCore::CostmapCore(CostmapConfig c) : config_(c) {
  if (!std::isfinite(c.resolution) || c.resolution <= 0 || !std::isfinite(c.size) ||
      c.size < 2*c.resolution || c.size/c.resolution > 2000 ||
      !std::isfinite(c.robot_radius) || c.robot_radius < 0 ||
      !std::isfinite(c.inflation_radius) || c.inflation_radius <= c.robot_radius ||
      c.inflation_radius > c.size/2) throw std::invalid_argument("Invalid costmap parameters");
}
Grid CostmapCore::build(const Scan& s) const {
  const int n = static_cast<int>(std::ceil(config_.size/config_.resolution));
  Grid g(n,n,config_.resolution,-n*config_.resolution/2,-n*config_.resolution/2);
  if (!std::isfinite(s.range_max) || s.range_max <= 0 ||
      !std::isfinite(s.angle_min) || !std::isfinite(s.angle_increment)) return g;
  std::vector<std::pair<int,int>> hits;
  // First clear only cells traversed by valid rays. Unobserved space stays -1.
  for (size_t i=0; i<s.ranges.size(); ++i) {
    const double r = s.ranges[i];
    if (std::isnan(r) || r < s.range_min || r == -INFINITY) continue;
    const double limit = std::min(s.range_max, config_.size/2-config_.resolution);
    const double length = std::min(r,limit);
    const double a = s.angle_min+i*s.angle_increment;
    for (double d=0; d<=length; d+=g.resolution/2) {
      int x,y;
      if (g.cell({d*std::cos(a), d*std::sin(a)},x,y)) g.data[g.index(x,y)]=0;
    }
    if (std::isfinite(r) && r < s.range_max && r <= limit) {
      int x,y;
      if (g.cell({r*std::cos(a),r*std::sin(a)},x,y)) hits.emplace_back(x,y);
    }
  }
  // Inflate after all ray clearing, so adjacent beams cannot erase a hit.
  // The extra cell diagonal covers endpoint quantization.
  const double hard = config_.robot_radius + std::sqrt(2.0)*g.resolution;
  const double outer = std::max(config_.inflation_radius,hard+g.resolution);
  const int radius=static_cast<int>(std::ceil(outer/g.resolution));
  for (const auto& hit:hits) {
    for (int dy=-radius;dy<=radius;++dy) for (int dx=-radius;dx<=radius;++dx) {
      const double d=std::hypot(dx,dy)*g.resolution;
      const int x=hit.first+dx,y=hit.second+dy;
      if (!g.contains(x,y) || d>outer) continue;
      // Reserve 100 for measured obstacles; 99 is footprint inflation. Keeping
      // them distinct allows bounded outward recovery from a rounded halo edge.
      const int cost=d<=hard ? 99 : std::clamp(static_cast<int>(98*(outer-d)/(outer-hard)),1,98);
      auto& cell=g.data[g.index(x,y)]; cell=static_cast<int8_t>(std::max<int>(cell,cost));
    }
  }
  for(const auto& hit:hits) g.data[g.index(hit.first,hit.second)]=100;
  return g;
}
}
