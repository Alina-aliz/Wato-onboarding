#pragma once
#include "navigation_common/types.hpp"
namespace robot {
class MapMemoryCore {
 public:
  MapMemoryCore(int cells=300, double resolution=0.2);
  bool integrate(const Grid& local, Pose sensor);
  const Grid& grid() const { return map_; }
 private:
  Grid map_;
};
}
