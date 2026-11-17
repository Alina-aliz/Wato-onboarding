#include "control_core.hpp"
#include <limits>
namespace robot {
ControlCore::ControlCore(ControlConfig c):config_(c) {
  for(double v:{c.lookahead,c.goal_tolerance,c.max_speed,c.max_angular,c.rotate_threshold})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid control parameters");
}
Velocity ControlCore::command(const std::vector<Point>& path,Pose p) const {
  if(path.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) return {};
  for(auto v:path) if(!std::isfinite(v.x)||!std::isfinite(v.y)) return {};
  const Point position{p.x,p.y};
  const double remaining=distance(position,path.back());
  if(remaining<=config_.goal_tolerance) return {};
  // Project onto the nearest segment, then walk lookahead metres along the path.
  size_t segment=0; Point projection=path.front();
  double best=std::numeric_limits<double>::infinity();
  for(size_t i=0;i+1<path.size();++i) {
    double dx=path[i+1].x-path[i].x,dy=path[i+1].y-path[i].y;
    double norm=dx*dx+dy*dy;
    double t=norm>0?std::clamp(((p.x-path[i].x)*dx+(p.y-path[i].y)*dy)/norm,0.0,1.0):0;
    Point q{path[i].x+t*dx,path[i].y+t*dy};
    if(distance(position,q)<best) {best=distance(position,q);segment=i;projection=q;}
  }
  Point target=path.back(),cursor=projection;
  double left=config_.lookahead;
  for(size_t i=segment+1;i<path.size();++i) {
    double length=distance(cursor,path[i]);
    if(length>=left && length>0) {
      target={cursor.x+(path[i].x-cursor.x)*left/length,cursor.y+(path[i].y-cursor.y)*left/length};
      break;
    }
    left-=length;cursor=path[i];
  }
  double dx=target.x-p.x,dy=target.y-p.y;
  double heading=wrap(std::atan2(dy,dx)-p.yaw);
  if(std::abs(heading)>config_.rotate_threshold)
    return {0,std::clamp(1.5*heading,-config_.max_angular,config_.max_angular)};
  double local_y=-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy;
  double curvature=2*local_y/std::max(dx*dx+dy*dy,1e-6);
  double speed=std::min({config_.max_speed,0.7*remaining,
                        config_.max_angular/std::max(std::abs(curvature),1e-6)});
  speed*=std::max(0.0,std::cos(heading));
  return {speed,std::clamp(speed*curvature,-config_.max_angular,config_.max_angular)};
}
}
