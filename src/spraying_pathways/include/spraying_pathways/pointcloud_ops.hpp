#pragma once
#include "spraying_pathways/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <set>
#include <string>
#include <cmath>

namespace sp {
namespace pc {

/** Extract XYZ points from a PointCloud2 (expects "x","y","z" fields). */
inline void points_from_pointcloud(const sensor_msgs::msg::PointCloud2& cloud,
                                   std::vector<geometry_msgs::msg::Point>& out)
{
  out.clear();
  out.reserve(static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height));

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    geometry_msgs::msg::Point p;
    p.x = *it_x; p.y = *it_y; p.z = *it_z;
    out.push_back(p);
  }
}

/** Return only points with z < (z_threshold - tolerance). */
inline std::vector<geometry_msgs::msg::Point>
filter_below_z(const std::vector<geometry_msgs::msg::Point>& in,
               double z_threshold, double tolerance = 1e-3)
{
  std::vector<geometry_msgs::msg::Point> out;
  out.reserve(in.size());
  for (const auto& p : in) {
    if (p.z < (z_threshold - tolerance)) out.push_back(p);
  }
  return out;
}

/** Build a minimal XYZ PointCloud2 from a list of geometry_msgs::Point. */
inline sensor_msgs::msg::PointCloud2
make_pointcloud_xyz(const std::vector<geometry_msgs::msg::Point>& points,
                    const std::string& frame_id,
                    const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp    = stamp;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

  for (const auto& p : points) {
    *it_x = p.x; *it_y = p.y; *it_z = p.z;
    ++it_x; ++it_y; ++it_z;
  }
  return cloud;
}

/** For each point, find the cube (axis-aligned) that contains it. De-duplicates by cube.id. */
inline std::vector<sp::Cube>
cubes_containing_points(const std::vector<sp::Cube>& cubes,
                        const std::vector<geometry_msgs::msg::Point>& points,
                        double cube_size_x, double cube_size_y)
{
  std::vector<sp::Cube> result;
  std::set<int> seen;
  const double hx = cube_size_x * 0.5;
  const double hy = cube_size_y * 0.5;

  for (const auto& p : points) {
    for (const auto& c : cubes) {
      const double dx = std::abs(c.pose.position.x - p.x);
      const double dy = std::abs(c.pose.position.y - p.y);
      if (dx <= hx && dy <= hy) {
        if (seen.insert(c.id).second) result.push_back(c);
        break;
      }
    }
  }
  return result;
}

/** Publish markers (spheres) at problematic cube centers. */
inline void publish_markers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
  const std::vector<sp::Cube>& cubes_with_issues,
  const std::string& frame_id,
  double cube_size_x, double cube_size_y,
  const rclcpp::Time& stamp)
{
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.reserve(cubes_with_issues.size());

  for (size_t i = 0; i < cubes_with_issues.size(); ++i) {
    const auto& c = cubes_with_issues[i];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp    = stamp;
    m.ns   = "problematic_cubes";
    m.id   = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = c.pose.position.x;
    m.pose.position.y = c.pose.position.y;
    m.pose.position.z = 0.05 + (c.height * 0.5);
    m.pose.orientation.w = 1.0;

    m.scale.x = cube_size_x * 0.5;
    m.scale.y = cube_size_y * 0.5;
    m.scale.z = 0.01;

    m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f; m.color.a = 0.9f;

    arr.markers.push_back(m);
  }
  pub->publish(arr);
}

} // namespace pc
} // namespace sp
