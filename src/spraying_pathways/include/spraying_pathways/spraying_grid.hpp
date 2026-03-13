#pragma once
#include "spraying_pathways/types.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sp {

// Order corners: [TL, TR, BR, BL]
// Input: any 4 points of an axis-aligned rectangle (unordered)
inline std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& pts) {
  if (pts.size() != 4) throw std::runtime_error("sort_rectangle_corners: exactly 4 points required");

  auto sorted = pts;
  // sort by y desc, then x asc
  std::sort(sorted.begin(), sorted.end(),
            [](const Point2D& a, const Point2D& b){
              if (a.y != b.y) return a.y > b.y;
              return a.x < b.x;
            });

  std::vector<Point2D> top(sorted.begin(),   sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin()+2, sorted.end());

  std::sort(top.begin(),    top.end(),    [](const Point2D& a, const Point2D& b){ return a.x < b.x; });
  std::sort(bottom.begin(), bottom.end(), [](const Point2D& a, const Point2D& b){ return a.x < b.x; });

  // [TL, TR, BR, BL]
  return { top[0], top[1], bottom[1], bottom[0] };
}

/**
 * Build a grid of spray patches over a rectangle.
 * - area_corners must be in order [TL, TR, BR, BL] (use sort_rectangle_corners first).
 * - Each patch has width step_x and length step_y.
 * - Inside each patch we create N "line cubes" stacked along Y (for a line pass).
 *   So: cube_size_x = step_x, cube_size_y = step_y / N.
 *
 * Outputs:
 * - cubes: centers at (patch center in X, evenly spaced in Y)
 * - spray_centers: one center per patch (zig-zag order by rows)
 * - cube_size_x, cube_size_y: sizes to use when visualizing cubes (SDF)
 */
inline void generate_grid(const std::vector<Point2D>& area_corners,
                          double spray_width, double spray_length, int N,
                          std::vector<Cube>& cubes,
                          std::vector<Point2D>& spray_centers,
                          double& cube_size_x, double& cube_size_y)
{
  if (area_corners.size() != 4) throw std::runtime_error("generate_grid: area_corners must have 4 points");

  const Point2D& TL = area_corners[0];
  const Point2D& TR = area_corners[1];
  const Point2D& BR = area_corners[2];
  const Point2D& BL = area_corners[3];

  // Components along rectangle axes (assumes axis-aligned or close to)
  const double dx = TR.x - TL.x;
  const double dy = BL.y - TL.y;

  // True edge lengths (robust even if slightly skewed)
  const double total_width  = std::hypot(TR.x - TL.x, TR.y - TL.y);
  const double total_length = std::hypot(BL.x - TL.x, BL.y - TL.y);

  int cols = std::max(1, static_cast<int>(total_width  / spray_width));
  int rows = std::max(1, static_cast<int>(total_length / spray_length));

  const double step_x = dx / cols;
  const double step_y = dy / rows;

  cube_size_x = step_x;
  cube_size_y = step_y / static_cast<double>(N);

  cubes.clear();
  spray_centers.clear();
  int cube_id = 0;

  for (int i = 0; i < rows; ++i) {
    std::vector<Point2D> row_centers;
    row_centers.reserve(cols);

    for (int j = 0; j < cols; ++j) {
      const double patch_origin_x = TL.x + j * step_x;
      const double patch_origin_y = TL.y + i * step_y;

      // N cube strips along Y inside the patch (line pass)
      for (int n = 0; n < N; ++n) {
        const double center_x = patch_origin_x + step_x * 0.5;
        const double center_y = patch_origin_y + (n + 0.5) * cube_size_y;
        cubes.push_back(Cube{ cube_id++, Pose3D{ {center_x, center_y}, 0.0 }, 0.0 });
      }

      // One spray center per patch (middle sub-strip)
      const int mid_idx = N / 2;
      const double waypoint_y = patch_origin_y + (mid_idx + 0.5) * cube_size_y;
      row_centers.push_back({ patch_origin_x + step_x * 0.5, waypoint_y });
    }

    // Zig-zag along rows
    if (i % 2 == 1) std::reverse(row_centers.begin(), row_centers.end());
    spray_centers.insert(spray_centers.end(), row_centers.begin(), row_centers.end());
  }
}

/**
 * Apply a flat-fan spray over the cubes:
 * - Only cubes within a narrow X band (epsilon ~ 1% of cube_size_x) around each spray line are affected
 * - Along Y within radius, height is added following a power law controlled by sigma
 * - cube.pose.z set to z_base + height/2 so SDF boxes sit on the base plane
 */
inline void apply_flat_spray(std::vector<Cube>& cubes,
                             const std::vector<Point2D>& spray_centers,
                             double cube_size_x, double cube_size_y,
                             double radius, double standard_h, double min_h,
                             double sigma, double z_base)
{
  const double epsilon = std::abs(cube_size_x) * 0.01; // narrow band around line

  for (const auto& sc : spray_centers) {
    for (auto& cube : cubes) {
      const double dx = std::abs(cube.pose.position.x - sc.x);
      const double dy = std::abs(cube.pose.position.y - sc.y);

      if (dy <= radius && dx <= epsilon) {
        const double h = (sigma == 0.0)
          ? standard_h
          : (standard_h - (standard_h - min_h) * std::pow(dy / radius, sigma));

        cube.height += h;
        cube.pose.z = z_base + cube.height * 0.5; // keep top at z_base + height
      }
    }
  }
}

} // namespace sp
