#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <algorithm>

// my shared headers
#include "spraying_pathways/types.hpp"           // sp::Point2D, sp::Pose3D, sp::Cube
#include "spraying_pathways/spraying_grid.hpp"   // sp::sort_rectangle_corners, sp::generate_grid, sp::apply_flat_spray
#include "spraying_pathways/multi_box_sdf.hpp"   // sp::generate_multi_box_sdf
#include "spraying_pathways/pointcloud_ops.hpp"  // sp::pc::{points_from_pointcloud, filter_below_z, ...}

// ------------------------------------------------------------------

std::vector<geometry_msgs::msg::Point> g_transformed_points;

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  sp::pc::points_from_pointcloud(*msg, g_transformed_points);
}

// ------------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spray_sim_node");

  auto sub_pc = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/transformed_points", rclcpp::SensorDataQoS(), pointCloudCallback);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/problematic_cubes_markers", 10);
  auto problematic_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/problematic_points", 10);

  using sp::Point2D;
  using sp::Cube;

  // Panel corners in WORLD (unordered)
  std::vector<Point2D> unordered = { {0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4} };
  std::vector<Point2D> corners = sp::sort_rectangle_corners(unordered);

  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  double z_base = 0.765;
  double spray_width  = 0.02;
  double spray_length = 0.10;
  int    N = 9;

  double radius = spray_length / 2.0;
  double standard_h = 0.015;
  double min_h      = 0.009;
  double sigma      = 3.0;

  double cube_size_x = 0.0, cube_size_y = 0.0;
  std::vector<Cube> cubes;
  std::vector<Point2D> spray_centers;

  // Build grid & apply spray
  sp::generate_grid(corners, spray_width, spray_length, N, cubes, spray_centers, cube_size_x, cube_size_y);
  cube_size_x = std::abs(cube_size_x);
  cube_size_y = std::abs(cube_size_y);
  sp::apply_flat_spray(cubes, spray_centers, cube_size_x, cube_size_y, radius, standard_h, min_h, sigma, z_base);

  // Plan the sweep path through spray centers
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  geometry_msgs::msg::Quaternion q; q.x=0; q.y=1; q.z=0; q.w=0;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(spray_centers.size());
  for (const auto& sc : spray_centers) {
    geometry_msgs::msg::Pose p;
    p.position.x = sc.x - robot_base_x;
    p.position.y = sc.y - robot_base_y;
    p.position.z = 0.2;
    p.orientation = q;
    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_thresh = 0.0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, trajectory);

  if (fraction > 0.90) {
    RCLCPP_INFO(node->get_logger(), "Planned %.1f%% of path. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan complete path (%.1f%%)", fraction * 100.0);
  }

  // Spawn SDF in Gazebo (WORLD frame)
  std::string sdf_content = sp::generate_multi_box_sdf(
      cubes, std::abs(cube_size_x), std::abs(cube_size_y),
      /*uri=*/"package://spraying_pathways/materials/scripts",
      /*mat=*/"My/Seaweed", /*static=*/true);

  std::ofstream out("/tmp/multi_cubes.sdf"); out << sdf_content; out.close();
  std::system("ros2 run gazebo_ros spawn_entity.py -file /tmp/multi_cubes.sdf -entity all_cubes");

  // Convert cubes to ROBOT BASE for matching with incoming points (which are in base_link)
  for (auto& c : cubes) {
    c.pose.position.x -= robot_base_x;
    c.pose.position.y -= robot_base_y;
  }

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    const auto now = node->now();

    // Points below threshold
    auto problematic = sp::pc::filter_below_z(g_transformed_points, 0.05 + standard_h);
    auto cloud_msg   = sp::pc::make_pointcloud_xyz(problematic, "base_link", now);
    problematic_cloud_pub->publish(cloud_msg);

    // Which cubes contain those points?
    auto cubes_with_issues = sp::pc::cubes_containing_points(cubes, problematic, cube_size_x, cube_size_y);
    sp::pc::publish_markers(marker_pub, cubes_with_issues, "base_link", cube_size_x, cube_size_y, now);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
