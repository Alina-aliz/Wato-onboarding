#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace robot {
struct Point { double x = 0.0, y = 0.0; };
struct Pose { double x = 0.0, y = 0.0, yaw = 0.0; };
struct Velocity { double linear = 0.0, angular = 0.0; };
inline double distance(Point a, Point b) { return std::hypot(a.x-b.x, a.y-b.y); }
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline Point transform(Point p, Pose t) {
  return {t.x + std::cos(t.yaw)*p.x - std::sin(t.yaw)*p.y,
          t.y + std::sin(t.yaw)*p.x + std::cos(t.yaw)*p.y};
}
struct Grid {
  int width = 0, height = 0;
  double resolution = 0.1, origin_x = 0.0, origin_y = 0.0;
  std::vector<int8_t> data;
  Grid() = default;
  Grid(int w, int h, double r, double x, double y)
      : width(w), height(h), resolution(r), origin_x(x), origin_y(y) {
    if (w <= 0 || h <= 0 || w > 4000 || h > 4000 || !std::isfinite(r) || r <= 0 ||
        !std::isfinite(x) || !std::isfinite(y)) throw std::invalid_argument("Invalid grid geometry");
    data.assign(static_cast<size_t>(w)*h, -1);
  }
  bool valid() const {
    return width > 0 && height > 0 && resolution > 0 && std::isfinite(resolution) &&
      std::isfinite(origin_x) && std::isfinite(origin_y) &&
      data.size() == static_cast<size_t>(width)*height;
  }
  bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  int index(int x, int y) const { return y*width+x; }
  bool cell(Point p, int& x, int& y) const {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || resolution <= 0) return false;
    const double gx = std::floor((p.x-origin_x)/resolution);
    const double gy = std::floor((p.y-origin_y)/resolution);
    if (gx < 0 || gy < 0 || gx >= width || gy >= height) return false;
    x = static_cast<int>(gx); y = static_cast<int>(gy); return true;
  }
  Point center(int x, int y) const {
    return {origin_x+(x+0.5)*resolution, origin_y+(y+0.5)*resolution};
  }
};
}  // namespace robot
