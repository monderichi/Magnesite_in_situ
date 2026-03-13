#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm>

// my shared headers
#include "spraying_pathways/types.hpp"
#include "spraying_pathways/spraying_grid.hpp"
#include "spraying_pathways/multi_box_sdf.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_path_planner");
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  using sp::Point2D;
  using sp::Cube;

  // MoveIt setup
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // --- Rectangle corners (unordered) in WORLD frame
  std::vector<Point2D> unordered_points = {
    {0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4}
  };
  auto corners = sp::sort_rectangle_corners(unordered_points);  // [TL, TR, BR, BL]

  // Robot base pose in WORLD frame (same as your spawn)
  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  // Spray & grid params
  double spray_width = 0.04;                 // width of each sweep (m)
  int    total_cubes_per_waypoint = 9;       // NxN cells inside each patch
  int    N = static_cast<int>(std::sqrt(total_cubes_per_waypoint));
  double z_base = 0.715 + 0.05;              // base Z of the panel

  // Height distribution params for cubes
  double max_height = 0.02;
  double random_height_min = 0.01;
  double random_height_max = max_height;

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;

  // --- Compute grid tiling over the rectangle
  const double dx = corners[1].x - corners[0].x;
  const double dy = corners[3].y - corners[0].y;

  const double total_len_x = std::hypot(dx,        corners[1].y - corners[0].y);
  const double total_len_y = std::hypot(corners[3].x - corners[0].x, dy);

  int cols = std::max(1, static_cast<int>(std::floor(total_len_x / spray_width)));
  int rows = std::max(1, static_cast<int>(std::floor(total_len_y / spray_width)));

  const double step_x = dx / cols;
  const double step_y = dy / rows;

  // Cube sizes per subcell (NxN inside each patch)
  const double cube_size_x = step_x / N;
  const double cube_size_y = step_y / N;

  std::vector<geometry_msgs::msg::Pose> waypoints;  // robot-base frame for planning
  std::vector<Cube> cubes;                          // world frame for Gazebo SDF
  int count = 0;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      // Zig-zag along X
      const int col = (i % 2 == 0) ? j : (cols - j - 1);

      // Center of this patch in WORLD frame
      const double cx_world = corners[0].x + (col + 0.5) * step_x;
      const double cy_world = corners[0].y + (i + 0.5) * step_y;

      // Waypoint in ROBOT BASE frame
      geometry_msgs::msg::Pose center_pose_base;
      center_pose_base.position.x = cx_world - robot_base_x;
      center_pose_base.position.y = cy_world - robot_base_y;
      center_pose_base.position.z = 0.2;
      center_pose_base.orientation = orientation;
      waypoints.push_back(center_pose_base);

      // Subdivide patch into NxN cubes (WORLD frame)
      const double start_x = cx_world - step_x / 2.0 + cube_size_x / 2.0;
      const double start_y = cy_world - step_y / 2.0 + cube_size_y / 2.0;

      for (int cx = 0; cx < N; ++cx) {
        for (int cy = 0; cy < N; ++cy) {
          const double x_pos = start_x + cx * cube_size_x;
          const double y_pos = start_y + cy * cube_size_y;

          // Center-weighted probabilistic height
          const double dx_c = cx - (N - 1) / 2.0;
          const double dy_c = cy - (N - 1) / 2.0;
          const double dist_norm = std::sqrt(dx_c * dx_c + dy_c * dy_c) / ((N - 1) / 2.0);
          const double prob_z1 = std::clamp(0.9 * (1.0 - dist_norm), 0.0, 0.9);
          const double rand_val = (std::rand() % 10000) / 10000.0;

          const double h = (rand_val < prob_z1)
            ? max_height
            : (random_height_min + ((std::rand() % 10000) / 10000.0) * (random_height_max - random_height_min));

          const double z_offset = z_base + h / 2.0;

          Cube c;
          c.id = count++;
          c.pose.position = {x_pos, y_pos};  // Point2D
          c.pose.z = z_offset;
          c.height = h;
          cubes.push_back(c);
        }
      }
    }
  }

  // Plan & execute a simple Cartesian path through centers
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.90) {
    RCLCPP_INFO(node->get_logger(), "Path %.2f%% planned successfully. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Path planning failed. Only %.2f%% completed.", fraction * 100.0);
  }

  // Build SDF with shared helper and spawn in Gazebo
  std::string sdf_all = sp::generate_multi_box_sdf(
      cubes,
      std::abs(cube_size_x),
      std::abs(cube_size_y),
      /*uri=*/"package://spraying_pathways/materials/scripts",
      /*mat=*/"My/Seaweed",
      /*static=*/true);

  const std::string path = "/tmp/multi_cubes.sdf";
  std::ofstream out(path); out << sdf_all; out.close();

  std::ostringstream cmd;
  cmd << "ros2 run gazebo_ros spawn_entity.py -file " << path << " -entity all_cubes";
  std::system(cmd.str().c_str());

  rclcpp::shutdown();
  return 0;
}
