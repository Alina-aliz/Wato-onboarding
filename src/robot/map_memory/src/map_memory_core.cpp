#include "map_memory_core.hpp"
namespace robot {
MapMemoryCore::MapMemoryCore(int cells,double resolution)
  : map_(cells,cells,resolution,-cells*resolution/2,-cells*resolution/2) {}
bool MapMemoryCore::integrate(const Grid& local,Pose sensor) {
  if (!local.valid() || !std::isfinite(sensor.x) || !std::isfinite(sensor.y) ||
      !std::isfinite(sensor.yaw)) return false;
  // Max-reduce each observation before merging: a free local cell must not
  // overwrite an occupied neighbor that projects into the same global cell.
  std::vector<int8_t> observation(map_.data.size(),-1);
  for (int y=0;y<local.height;++y) for(int x=0;x<local.width;++x) {
    int value=local.data[local.index(x,y)];
    if(value<0) continue;
    int gx,gy;
    if(map_.cell(transform(local.center(x,y),sensor),gx,gy)) {
      auto& target=observation[map_.index(gx,gy)];
      target=static_cast<int8_t>(std::max<int>(target,value));
    }
  }
  bool changed=false;
  for(size_t i=0;i<observation.size();++i) {
    if(observation[i]>=0 && observation[i]!=map_.data[i]) {
      map_.data[i]=observation[i]; changed=true;
    }
  }
  return changed;
}
}
