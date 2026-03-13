#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>

// my shared headers
#include "spraying_pathways/types.hpp"           // sp::Point2D, sp::Pose3D, sp::Cube
#include "spraying_pathways/spraying_grid.hpp"   // sp::sort_rectangle_corners, sp::generate_grid, sp::apply_flat_spray
#include "spraying_pathways/multi_box_sdf.hpp"   // sp::generate_multi_box_sdf
#include "spraying_pathways/spray_trajectory.hpp"           // sp::traj::{organize_waypoints_into_spray_lines, generate_multi_line_trajectory_quintic, nearest_time_at_xy}

using sp::Point2D;
using sp::Cube;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = rclcpp::Node::make_shared("spray_sim_node", options);

  // Run executor in a thread so MoveGroup can use the node
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor](){ executor->spin(); });

  // === Load kinematics.yaml into parameters
  try {
    std::string kinematics_path =
      ament_index_cpp::get_package_share_directory("ur_moveit_config") + "/config/kinematics.yaml";
    YAML::Node yaml = YAML::LoadFile(kinematics_path);
    if (yaml["/**"] && yaml["/**"]["ros__parameters"] &&
        yaml["/**"]["ros__parameters"]["robot_description_kinematics"]) {
      auto kinematics = yaml["/**"]["ros__parameters"]["robot_description_kinematics"];
      for (auto group : kinematics) {
        std::string group_name = group.first.as<std::string>();
        for (auto param : group.second) {
          std::string param_name = "robot_description_kinematics." + group_name + "." + param.first.as<std::string>();
          auto value = param.second;
          if (value.IsScalar()) {
            try { node->declare_parameter(param_name, value.as<double>()); }
            catch (...) {
              try { node->declare_parameter(param_name, value.as<int>()); }
              catch (...) { node->declare_parameter(param_name, value.as<std::string>()); }
            }
          }
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "Failed to load kinematics.yaml: %s", e.what());
  }

  // Rectangle corners (unordered)
  std::vector<Point2D> unordered = { {0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4} };
  std::vector<Point2D> corners = sp::sort_rectangle_corners(unordered);

  // Robot base in WORLD
  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  // Spray/grid params
  double z_base = 0.765;
  double spray_width  = 0.02;
  double spray_length = 0.10;
  int    N = 9;

  // Spray law params
  double radius = spray_length / 2.0;
  double standard_h = 0.015;
  double min_h      = 0.009;
  double sigma      = 3.0;

  double cube_size_x = 0.0, cube_size_y = 0.0;
  std::vector<Cube> cubes;
  std::vector<Point2D> spray_centers;

  // Build grid + centers and apply spray
  sp::generate_grid(corners, spray_width, spray_length, N, cubes, spray_centers, cube_size_x, cube_size_y);
  cube_size_x = std::abs(cube_size_x);
  cube_size_y = std::abs(cube_size_y);
  sp::apply_flat_spray(cubes, spray_centers, cube_size_x, cube_size_y, radius, standard_h, min_h, sigma, z_base);

  // Build waypoints in robot base frame
  geometry_msgs::msg::Quaternion q; q.x=0; q.y=1; q.z=0; q.w=0;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (const auto& sc : spray_centers) {
    geometry_msgs::msg::Pose p;
    p.position.x = sc.x - robot_base_x;
    p.position.y = sc.y - robot_base_y;
    p.position.z = 0.2;
    p.orientation = q;
    waypoints.push_back(p);
  }

  // Group into spray lines & print
  auto spray_lines = sp::traj::organize_waypoints_into_spray_lines(waypoints);
  for (size_t i = 0; i < spray_lines.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("spray_logger"), "Spray line %zu: %zu points", i, spray_lines[i].size());
    for (size_t j = 0; j < spray_lines[i].size(); ++j)
      RCLCPP_INFO(rclcpp::get_logger("spray_logger"), "  Point %zu: (%.3f, %.3f)",
                  j, spray_lines[i][j].x, spray_lines[i][j].y);
  }

  // Generate continuous multi-line trajectory (quintic transitions)
  auto [t, xy, v, a] = sp::traj::generate_multi_line_trajectory_quintic(spray_lines, 0.08, 0.03, 100);

  // Save trajectory CSV
  {
    std::ofstream file("trajectory_output.csv");
    if (file.is_open()) {
      file << "time [s],x [m],y [m],speed [m/s],acceleration [m/s²]\n";
      for (size_t i = 0; i < t.size(); ++i) {
        file << t[i] << "," << xy[i].x << "," << xy[i].y << "," << v[i] << "," << a[i] << "\n";
        RCLCPP_INFO(node->get_logger(), "[Time %.3f s]  x[%f] y[%f] v[%f] a[%f]", t[i], xy[i].x, xy[i].y, v[i], a[i]);
      }
      RCLCPP_INFO(node->get_logger(), "Trajectory saved to 'trajectory_output.csv'");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to write to 'trajectory_output.csv'");
    }
  }

  RCLCPP_INFO(node->get_logger(), "[First point] x=%.3f y=%.3f", xy.front().x, xy.front().y);

  // Move to first trajectory point
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = xy.front().x;
  target_pose.position.y = xy.front().y;
  target_pose.position.z = 0.2;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 1.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  moveit_msgs::msg::RobotTrajectory traj_first;
  double fraction = move_group.computeCartesianPath({target_pose}, eef_step, jump_threshold, traj_first);
  if (fraction > 0.95) {
    RCLCPP_INFO(node->get_logger(), "Cartesian path to first point planned (%.1f%%).", fraction*100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan; plan.trajectory_ = traj_first;
    move_group.execute(plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Only %.1f%% of Cartesian path to first point was planned", fraction*100.0);
  }

  // Plan full path through all generated waypoints (constructed from xy)
  std::vector<geometry_msgs::msg::Pose> trajectory_waypoints;
  trajectory_waypoints.reserve(xy.size());
  for (const auto& pxy : xy) {
    geometry_msgs::msg::Pose p;
    p.position.x = pxy.x; p.position.y = pxy.y; p.position.z = 0.2;
    p.orientation.x = 0.0; p.orientation.y = 1.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
    trajectory_waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_2;
  double fraction_2 = move_group.computeCartesianPath(trajectory_waypoints, eef_step, jump_threshold, trajectory_2);
  RCLCPP_INFO(node->get_logger(), "Cartesian path planning completed %.2f%% of the path", fraction_2*100.0);

  // FK logging & time mapping
  const std::string planning_group = move_group.getName();
  const std::string eef_link = move_group.getEndEffectorLink();

  moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup* jmg = kinematic_state->getJointModelGroup(planning_group);

  const auto& joint_names = trajectory_2.joint_trajectory.joint_names;
  const auto& joint_points = trajectory_2.joint_trajectory.points;

  RCLCPP_INFO(node->get_logger(), "Extracted %zu joint points from trajectory.", joint_points.size());

  for (size_t i = 0; i < joint_points.size(); ++i) {
    const auto& point = joint_points[i];
    RCLCPP_INFO(node->get_logger(), "Waypoint %zu:", i);
    for (size_t j = 0; j < point.positions.size(); ++j)
      RCLCPP_INFO(node->get_logger(), "  %s: %.6f", joint_names[j].c_str(), point.positions[j]);
  }

  robot_trajectory::RobotTrajectory robot_traj(move_group.getRobotModel(), planning_group);
  double last_time = 0.0;

  for (size_t i = 0; i < joint_points.size(); ++i) {
    kinematic_state->setJointGroupPositions(jmg, joint_points[i].positions);
    const Eigen::Isometry3d& tf = kinematic_state->getGlobalLinkTransform(eef_link);
    Point2D query_xy{tf.translation().x(), tf.translation().y()};

    double matched_time = sp::traj::nearest_time_at_xy(xy, t, query_xy);
    double delta_time = matched_time - last_time;
    last_time = matched_time;

    robot_traj.addSuffixWayPoint(*kinematic_state, std::max(0.0, delta_time));
    RCLCPP_INFO(node->get_logger(),
      "[Waypoint %zu] FK xy(%.4f, %.4f) -> t_from_start=%.3f -> dt=%.3f",
      i, query_xy.x, query_xy.y, matched_time, delta_time);
  }

  moveit_msgs::msg::RobotTrajectory msg;
  robot_traj.getRobotTrajectoryMsg(msg);
  moveit::planning_interface::MoveGroupInterface::Plan plan; plan.trajectory_ = msg;
  RCLCPP_INFO(node->get_logger(), "Executing FK-time-mapped trajectory...");
  move_group.execute(plan);

  // Spawn cubes SDF in Gazebo
  std::string sdf_content = sp::generate_multi_box_sdf(cubes, std::abs(cube_size_x), std::abs(cube_size_y));
  std::ofstream out("/tmp/multi_cubes.sdf"); out << sdf_content; out.close();
  std::system("ros2 run gazebo_ros spawn_entity.py -file /tmp/multi_cubes.sdf -entity all_cubes");

  executor->cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
