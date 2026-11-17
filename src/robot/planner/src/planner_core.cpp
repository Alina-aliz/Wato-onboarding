#include "planner_core.hpp"
#include <limits>
#include <queue>
#include <functional>
#include <utility>
namespace robot {
PlannerCore::PlannerCore(PlannerConfig c):config_(c) {
  if(c.obstacle_threshold<1 || c.obstacle_threshold>100 || !std::isfinite(c.unknown_cost) ||
     c.unknown_cost<1 || !std::isfinite(c.cost_weight) || c.cost_weight<0)
    throw std::invalid_argument("Invalid planner parameters");
}
bool PlannerCore::traversable(const Grid& g,int x,int y) const {
  return g.contains(x,y) && g.data[g.index(x,y)]<config_.obstacle_threshold;
}
bool PlannerCore::segmentClear(const Grid& g,Point a,Point b) const {
  if(!g.valid()) return false;
  const int steps=std::max(1,static_cast<int>(std::ceil(distance(a,b)/(g.resolution/3))));
  int previous_x=-1,previous_y=-1;
  for(int i=0;i<=steps;++i) {
    double t=static_cast<double>(i)/steps;
    int x,y;
    if(!g.cell({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)},x,y) || !traversable(g,x,y)) return false;
    if(previous_x>=0 && x!=previous_x && y!=previous_y &&
       (!traversable(g,x,previous_y)||!traversable(g,previous_x,y))) return false;
    previous_x=x; previous_y=y;
  }
  return true;
}
bool PlannerCore::pathUsable(const Grid& g,const std::vector<Point>& path,Point position) const {
  if(path.empty() || !g.valid()) return false;
  size_t nearest=0;
  double best=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<path.size();++i) {
    const double d=distance(position,path[i]);
    if(d<best) {best=d;nearest=i;}
  }
  // Check only the untraversed suffix. New obstacles behind the robot must not
  // make it discard a valid route and choose the other side of an obstacle.
  if(best>0.75 || !segmentClear(g,position,path[nearest])) return false;
  for(size_t i=nearest+1;i<path.size();++i)
    if(!segmentClear(g,path[i-1],path[i])) return false;
  return true;
}
std::vector<Point> PlannerCore::plan(const Grid& g,Point start,Point goal,
                                    const std::vector<Point>& preferred_path) const {
  int sx,sy,gx,gy;
  if(!g.valid() || !g.cell(start,sx,sy) || !g.cell(goal,gx,gy) ||
     !traversable(g,gx,gy)) return {};
  if(!traversable(g,sx,sy)) {
    // A map can move the conservative inflation boundary over the robot due
    // to discretization/reprojection. Never clear a measured obstacle (100).
    // Only relax nearby inflation (99), only outward from observed obstacles,
    // and only in a small region connected to the current pose by A*.
    if(config_.obstacle_threshold!=99 || g.data[g.index(sx,sy)]!=99) return {};
    std::vector<Point> obstacles;
    const int search=static_cast<int>(std::ceil(4.0/g.resolution));
    for(int y=std::max(0,sy-search);y<=std::min(g.height-1,sy+search);++y)
      for(int x=std::max(0,sx-search);x<=std::min(g.width-1,sx+search);++x)
        if(g.data[g.index(x,y)]==100) obstacles.push_back(g.center(x,y));
    if(obstacles.empty()) return {};
    auto clearance=[&](Point p) {
      double nearest=std::numeric_limits<double>::infinity();
      for(Point obstacle:obstacles) nearest=std::min(nearest,distance(p,obstacle));
      return nearest;
    };
    const double initial_clearance=clearance(g.center(sx,sy));
    Grid recovery=g;
    const int radius=static_cast<int>(std::ceil(0.6/g.resolution));
    for(int y=std::max(0,sy-radius);y<=std::min(g.height-1,sy+radius);++y)
      for(int x=std::max(0,sx-radius);x<=std::min(g.width-1,sx+radius);++x) {
        Point p=g.center(x,y);
        if(g.data[g.index(x,y)]==99 && distance(p,g.center(sx,sy))<=0.6 &&
           clearance(p)+1e-9>=initial_clearance) recovery.data[g.index(x,y)]=98;
      }
    return plan(recovery,start,goal,preferred_path);
  }
  int source=g.index(sx,sy),target=g.index(gx,gy);
  if(source==target) return segmentClear(g,start,goal) ? std::vector<Point>{start,goal} : std::vector<Point>{};
  std::vector<double> scores(g.data.size(),std::numeric_limits<double>::infinity());
  std::vector<int> parent(g.data.size(),-1);
  std::vector<bool> closed(g.data.size(),false);
  std::vector<double> preference_cost(g.data.size(),-1.0);
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  auto h=[&](int x,int y){return std::hypot(x-gx,y-gy);};
  scores[source]=0; open.emplace(h(sx,sy),source);
  while(!open.empty()) {
    const int current=open.top().second; open.pop();
    if(closed[current]) continue;
    if(current==target) {
      std::vector<Point> path;
      for(int p=target;p!=-1;p=parent[p]) path.push_back(g.center(p%g.width,p/g.width));
      std::reverse(path.begin(),path.end());
      path.front()=start; path.back()=goal;
      // Preserve the grid path; unchecked smoothing can cut obstacle corners.
      for(size_t i=1;i<path.size();++i) if(!segmentClear(g,path[i-1],path[i])) return {};
      return path;
    }
    closed[current]=true;
    const int x=current%g.width,y=current/g.width;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
      if(dx==0 && dy==0) continue;
      int nx=x+dx,ny=y+dy;
      if(!traversable(g,nx,ny)) continue;
      if(dx!=0 && dy!=0 && (!traversable(g,x+dx,y)||!traversable(g,x,y+dy))) continue;
      int next=g.index(nx,ny),value=g.data[next];
      double weight=value<0?config_.unknown_cost:1+config_.cost_weight*value/100.0;
      if(!preferred_path.empty()) {
        double& penalty=preference_cost[next];
        if(penalty<0) {
          double nearest=std::numeric_limits<double>::infinity();
          Point location=g.center(nx,ny);
          for(Point waypoint:preferred_path) nearest=std::min(nearest,distance(location,waypoint));
          penalty=6.0*std::min(nearest,2.0);
        }
        weight+=penalty;
      }
      double candidate=scores[current]+std::hypot(dx,dy)*weight;
      if(candidate<scores[next]) {
        parent[next]=current; scores[next]=candidate;
        open.emplace(candidate+h(nx,ny),next);
      }
    }
  }
  return {};
}
}
