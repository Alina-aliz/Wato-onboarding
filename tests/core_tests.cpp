#include "costmap_core.hpp"
#include "map_memory_core.hpp"
#include "planner_core.hpp"
#include "control_core.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {
  if(!condition) throw std::runtime_error(message);
}
int value(const robot::Grid& grid,robot::Point p) {
  int x=0,y=0;
  if(!grid.cell(p,x,y)) throw std::runtime_error("test point outside grid");
  return grid.data[grid.index(x,y)];
}
void costmapTests() {
  robot::CostmapCore core({0.1,12.0,0.3,0.8});
  robot::Scan scan{0,0.01,0.08,5.0,{3.0f}};
  auto map=core.build(scan);
  require(value(map,{3,0})==100,"scan endpoint must be occupied");
  require(value(map,{1,0})==0,"traversed ray must be free");
  require(value(map,{-1,0})==-1,"unseen space must stay unknown");
  require(value(map,{3.2,0})==99,"footprint inflation must be distinct from measured obstacles");
  require(value(map,{3.6,0})>0 && value(map,{3.6,0})<99,"outer inflation must decay");
  require(value(map,{4,0})==-1,"space behind hit must not be cleared");
  scan.ranges={std::numeric_limits<float>::infinity()};
  map=core.build(scan);
  require(value(map,{4,0})==0,"positive infinity must clear to maximum range");
  require(value(map,{5.1,0})==-1,"no return must not create a maximum-range obstacle");
  scan.ranges={std::numeric_limits<float>::quiet_NaN(),-1.0f};
  map=core.build(scan);
  for(int8_t cell:map.data) require(cell==-1,"invalid rays must not create observations");
}
void memoryTests() {
  robot::MapMemoryCore memory(100,0.2);
  robot::Grid local(2,2,0.1,0,0);
  local.data={100,0,-1,-1};
  require(memory.integrate(local,{1,2,0}),"first observation must change map");
  require(value(memory.grid(),{1.05,2.05})==100,"max reduction must retain hit over free cells");
  local.data.assign(4,-1);
  memory.integrate(local,{1,2,0});
  require(value(memory.grid(),{1.05,2.05})==100,"unknown must retain remembered obstacle");
  robot::Grid rotated(1,1,0.2,0,0);rotated.data={100};
  memory.integrate(rotated,{3,2,1.5707963267948966});
  require(value(memory.grid(),{2.9,2.1})==100,"rotated scan must land in global coordinates");
  local.data.assign(4,0);memory.integrate(local,{1,2,0});
  require(value(memory.grid(),{1.05,2.05})==0,"fresh observed free space must replace old data");
  memory.integrate(local,{1000,1000,0}); // Out-of-map updates must be safe.
}
void plannerTests() {
  robot::PlannerCore planner;
  robot::Grid grid(20,20,1,0,0);grid.data.assign(400,0);
  for(int y=0;y<20;++y) if(y!=10) grid.data[grid.index(10,y)]=100;
  auto path=planner.plan(grid,{2.5,2.5},{17.5,2.5});
  require(!path.empty(),"A* must route through wall opening");
  bool opening=false;
  for(auto p:path) if(p.x==10.5 && p.y==10.5) opening=true;
  require(opening,"wall route must pass through the opening");
  for(size_t i=1;i<path.size();++i)
    require(planner.segmentClear(grid,path[i-1],path[i]),"every planned segment must be collision free");
  grid.data[grid.index(10,10)]=100;
  require(planner.plan(grid,{2.5,2.5},{17.5,2.5}).empty(),"sealed wall must have no path");
  require(planner.plan(grid,{-1,0},{1,1}).empty(),"out-of-bounds start must be rejected");
  require(planner.plan(grid,{1,1},{10.5,2.5}).empty(),"occupied goal must be rejected");
  robot::Grid corner(2,2,1,0,0);corner.data={0,100,100,0};
  require(planner.plan(corner,{0.5,0.5},{1.5,1.5}).empty(),"diagonal corner cutting must be forbidden");
  robot::Grid unknown(5,5,1,0,0);
  require(!planner.plan(unknown,{0.5,0.5},{4.5,4.5}).empty(),"unknown cells must allow exploratory planning");
  require(planner.plan(robot::Grid{}, {0,0},{1,1}).empty(),"invalid grid must be rejected");
  robot::Grid halo(50,50,0.2,-5,-5);halo.data.assign(2500,0);
  const robot::Point obstacle=halo.center(25,25);
  for(int y=0;y<50;++y) for(int x=0;x<50;++x)
    if(robot::distance(halo.center(x,y),obstacle)<2.0) halo.data[halo.index(x,y)]=99;
  halo.data[halo.index(25,25)]=100;
  const robot::Point edge=halo.center(34,25);
  auto escape=planner.plan(halo,edge,{4.1,0.1});
  require(!escape.empty(),"robot on rounded inflation edge must be able to exit outward");
  for(auto p:escape) require(robot::distance(p,obstacle)+1e-6>=robot::distance(edge,obstacle),
    "halo recovery must not move closer to the obstacle");
  require(planner.plan(halo,obstacle,{4.1,0.1}).empty(),"measured obstacle start must never be cleared");
  require(planner.plan(halo,{4.1,0.1},edge).empty(),"inflated goal must remain blocked");
  robot::Grid stable(12,12,1,0,0);stable.data.assign(144,0);
  const std::vector<robot::Point> route{{1.5,1.5},{2.5,1.5},{3.5,1.5},{4.5,1.5}};
  require(planner.pathUsable(stable,route,{2.5,1.5}),"clear route must be retained");
  stable.data[stable.index(3,1)]=80;
  require(planner.pathUsable(stable,route,{2.5,1.5}),"soft cost changes must not switch routes");
  stable.data[stable.index(1,1)]=100;
  require(planner.pathUsable(stable,route,{2.5,1.5}),"obstacle behind robot must not invalidate suffix");
  stable.data[stable.index(3,1)]=100;
  require(!planner.pathUsable(stable,route,{2.5,1.5}),"new obstacle ahead must require replanning");
  require(!planner.pathUsable(stable,route,{10.5,10.5}),"large path deviation must require replanning");
  robot::Grid preference_grid(6,6,1,0,0);preference_grid.data.assign(36,0);
  std::vector<robot::Point> preferred;
  for(int y=0;y<=3;++y) preferred.push_back({0.5,y+0.5});
  for(int x=1;x<=4;++x) preferred.push_back({x+0.5,3.5});
  for(int y=2;y>=0;--y) preferred.push_back({4.5,y+0.5});
  auto committed=planner.plan(preference_grid,{0.5,0.5},{4.5,0.5},preferred);
  require(!committed.empty(),"preferred route must remain plannable");
  double highest=0;
  for(auto p:committed) highest=std::max(highest,p.y);
  require(highest>=2.5,"replanning must favor the previously selected corridor");
  preference_grid.data[preference_grid.index(2,3)]=100;
  auto detour=planner.plan(preference_grid,{0.5,0.5},{4.5,0.5},preferred);
  require(!detour.empty(),"route preference must permit a detour around new obstacles");
  for(auto p:detour) require(value(preference_grid,p)<99,"preference must never override collisions");
}
void controlTests() {
  robot::ControlCore control;
  require(control.command({},{}).linear==0,"empty path must stop");
  auto straight=control.command({{0,0},{5,0}},{0,0,0});
  require(straight.linear>0 && std::abs(straight.angular)<1e-9,"straight path must drive straight");
  auto behind=control.command({{0,0},{-5,0}},{0,0,0});
  require(behind.linear==0 && std::abs(behind.angular)>0,"target behind must rotate before driving");
  auto left=control.command({{0,0},{3,1}},{0,0,0});
  require(left.angular>0,"target on left must produce positive yaw rate");
  require(control.command({{0,0},{0.1,0}},{0,0,0}).linear==0,"goal tolerance must stop robot");
  auto invalid=control.command({{NAN,0}},{0,0,0});
  require(invalid.linear==0 && invalid.angular==0,"nonfinite path must stop");
  // Closed-loop differential-drive integration checks convergence, not only signs.
  const std::vector<robot::Point> path{{0,0},{2,0},{3,1},{3,3}};
  robot::Pose pose{0,0,0};
  for(int i=0;i<3000;++i) {
    auto v=control.command(path,pose);
    require(v.linear>=0 && v.linear<=0.4 && std::abs(v.angular)<=0.6,"commands must respect limits");
    pose.x+=v.linear*std::cos(pose.yaw)*0.05;
    pose.y+=v.linear*std::sin(pose.yaw)*0.05;
    pose.yaw=robot::wrap(pose.yaw+v.angular*0.05);
  }
  require(robot::distance({pose.x,pose.y},path.back())<=0.26,"controller must converge around turns");
}
}
int main() {
  try {
    costmapTests();memoryTests();plannerTests();controlTests();
    std::cout<<"PASS: costmap, map memory, A*, and closed-loop controller tests\n";
  } catch(const std::exception& e) {
    std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;
  }
}
