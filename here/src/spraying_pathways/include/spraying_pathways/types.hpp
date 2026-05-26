#pragma once
#include <cmath>
#include <vector>

namespace sp {

struct Point2D {
  double x{0}, y{0};
  bool operator<(const Point2D& o) const { return (y != o.y) ? (y > o.y) : (x < o.x); }
};

struct Pose3D { Point2D position; double z{0}; };

struct Cube { int id{0}; Pose3D pose; double height{0}; };

} // namespace sp
