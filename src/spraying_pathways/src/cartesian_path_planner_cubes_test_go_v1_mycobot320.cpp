#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>

using Point2D = std::pair<double, double>;

struct Cube {
  geometry_msgs::msg::Pose pose;
  double height;
  int id;
};

double round_to(double value, int decimals) {
  double factor = std::pow(10.0, decimals);
  return std::round(value * factor) / factor;
}

std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& points) {
  if (points.size() != 4)
    throw std::runtime_error("Exactly 4 points are required.");

  auto sorted = points;
  std::sort(sorted.begin(), sorted.end(), [](const Point2D& a, const Point2D& b) {
    return a.second > b.second || (a.second == b.second && a.first < b.first);
  });

  std::vector<Point2D> top(sorted.begin(), sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin() + 2, sorted.end());

  std::sort(top.begin(), top.end());
  std::sort(bottom.begin(), bottom.end());

  return {top[0], top[1], bottom[1], bottom[0]};
}

std::string generate_multi_box_sdf(const std::vector<Cube>& cubes, double cube_size_x, double cube_size_y) {
  std::ostringstream sdf;
  sdf << "<?xml version='1.0'?>\n";
  sdf << "<sdf version='1.7'>\n";
  sdf << "<model name='multi_cube'>\n";
  sdf << "  <static>true</static>\n";
  for (const auto& cube : cubes) {
    sdf << "  <link name='cube_" << cube.id << "'>\n";
    sdf << "    <pose>"
        << cube.pose.position.x << " "
        << cube.pose.position.y << " "
        << cube.pose.position.z << " 0 0 0</pose>\n";
    sdf << "    <collision name='collision'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>" << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n";
    sdf << "    </collision>\n";
    sdf << "    <visual name='visual'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>" << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n";
    sdf << "      <material>\n";
    sdf << "        <ambient>0.2 0.8 0.2 0.7</ambient>\n";
    sdf << "        <diffuse>0.2 0.8 0.2 0.7</diffuse>\n";
    sdf << "      </material>\n";
    sdf << "    </visual>\n";
    sdf << "  </link>\n";
  }
  sdf << "</model>\n";
  sdf << "</sdf>\n";
  return sdf.str();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_path_planner_mycobot320");
  std::srand(std::time(nullptr));

  RCLCPP_INFO(node->get_logger(), "============================================");
  RCLCPP_INFO(node->get_logger(), "myCobot 320 Cartesian Path Planner (Cubes)");
  RCLCPP_INFO(node->get_logger(), "============================================");

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(30.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Move to 'ready' position first for a known starting point
  RCLCPP_INFO(node->get_logger(), "Moving to 'ready' position first...");
  move_group.setNamedTarget("ready");
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
  bool ready_success = (move_group.plan(ready_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (ready_success) {
    RCLCPP_INFO(node->get_logger(), "Ready position plan successful, executing...");
    move_group.execute(ready_plan);
    RCLCPP_INFO(node->get_logger(), "At ready position.");
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not plan to 'ready' position, trying from current position...");
  }

  // Reset planning params for Cartesian path
  move_group.setPlanningTime(30.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // ==========================================
  // myCobot 320 workspace configuration
  // ==========================================
  // Robot is spawned at world (0.25, 0, 0.715) in Gazebo
  // myCobot 320 reach: ~320mm from base center
  // MoveIt operates in robot base_link frame (origin at robot base)
  //
  // Safe reachable workspace (in robot frame):
  //   X: 0.15 to 0.28 (forward from base)
  //   Y: -0.08 to 0.08 (left-right)
  //   Z: keep it reasonable above table
  //
  // World coordinates for cubes: add robot_base offset (0.25, 0, 0.715)
  // ==========================================

  // Rectangle in WORLD coordinates (for cube spawning)
  // These represent the spray area on the table
  const double robot_base_x = 0.25;  // Robot spawn X in Gazebo
  const double robot_base_y = 0.0;   // Robot spawn Y in Gazebo
  const double robot_base_z = 0.715; // Robot spawn Z in Gazebo (table height)

  // Define rectangle corners in WORLD coordinates
  std::vector<Point2D> unordered_points = {
    {0.40, -0.08},  // front-left
    {0.40,  0.08},  // front-right
    {0.53, -0.08},  // back-left
    {0.53,  0.08}   // back-right
  };

  auto corners = sort_rectangle_corners(unordered_points);
  RCLCPP_INFO(node->get_logger(), "Rectangle corners in world frame (TL, TR, BR, BL):");
  RCLCPP_INFO(node->get_logger(), "  TL: (%.3f, %.3f)", corners[0].first, corners[0].second);
  RCLCPP_INFO(node->get_logger(), "  TR: (%.3f, %.3f)", corners[1].first, corners[1].second);
  RCLCPP_INFO(node->get_logger(), "  BR: (%.3f, %.3f)", corners[2].first, corners[2].second);
  RCLCPP_INFO(node->get_logger(), "  BL: (%.3f, %.3f)", corners[3].first, corners[3].second);

  double spray_width = 0.03;      // 3cm spacing (finer for small robot)
  int total_cubes_per_waypoint = 9;
  int N = static_cast<int>(std::sqrt(total_cubes_per_waypoint));

  double z_base = robot_base_z + 0.01;  // Just above table surface for cubes
  double max_height = 0.015;             // Max cube height
  double z_height = 0.20;               // End-effector height above table in robot frame

  // Orientation: straight down for best reachability
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;

  auto dx = corners[1].first - corners[0].first;
  auto dy = corners[3].second - corners[0].second;

  double length = std::hypot(dx, corners[1].second - corners[0].second);
  double height = std::hypot(corners[3].first - corners[0].first, dy);

  int cols = std::max(1, static_cast<int>(std::floor(length / spray_width)));
  int rows = std::max(1, static_cast<int>(std::floor(height / spray_width)));

  double step_x = dx / cols;
  double step_y = dy / rows;

  double cube_size_x = step_x / N;
  double cube_size_y = step_y / N;

  RCLCPP_INFO(node->get_logger(), "Spray grid: %d rows x %d cols, step=(%.3f, %.3f)", rows, cols, step_x, step_y);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<std::tuple<double, double>> positions;  // World positions for cubes

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int col = (i % 2 == 0) ? j : (cols - j - 1);  // Zig-zag

      // Waypoint in ROBOT FRAME (subtract robot base offset)
      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x = corners[0].first + (col + 0.5) * step_x - robot_base_x;
      center_pose.position.y = corners[0].second + (i + 0.5) * step_y - robot_base_y;
      center_pose.position.z = z_height;
      center_pose.orientation = orientation;

      waypoints.push_back(center_pose);

      RCLCPP_INFO(node->get_logger(), "Waypoint [%zu]: robot_frame(%.3f, %.3f, %.3f)",
                  waypoints.size()-1, center_pose.position.x, center_pose.position.y, center_pose.position.z);

      // Cube positions in WORLD FRAME
      double start_x = corners[0].first + (col + 0.5) * step_x - step_x / 2.0 + cube_size_x / 2.0;
      double start_y = corners[0].second + (i + 0.5) * step_y - step_y / 2.0 + cube_size_y / 2.0;

      for (int cx = 0; cx < N; ++cx) {
        for (int cy = 0; cy < N; ++cy) {
          double x_pos = start_x + cx * cube_size_x;
          double y_pos = start_y + cy * cube_size_y;
          positions.push_back({round_to(x_pos, 4), round_to(y_pos, 4)});
        }
      }
    }
  }

  // Show available cube positions
  std::map<std::pair<double, double>, double> selected_positions;
  std::cout << "\nAvailable positions (x y) in world frame:\n";
  for (const auto& pos : positions) {
    std::cout << std::fixed << std::setprecision(4) << std::get<0>(pos) << " " << std::get<1>(pos) << "\n";
  }

  std::cout << "\nEnter positions and height (x y h), type 'done' to finish:\n";
  std::string input;
  while (true) {
    std::getline(std::cin, input);
    if (input == "done") break;
    std::istringstream iss(input);
    double x, y, h;
    if (iss >> x >> y >> h) {
      selected_positions[{round_to(x, 4), round_to(y, 4)}] = h;
    }
  }

  // Execute spray trajectory (Cartesian path)
  RCLCPP_INFO(node->get_logger(), "Planning Cartesian path through %zu waypoints...", waypoints.size());
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.9) {
    RCLCPP_INFO(node->get_logger(), "Path %.2f%% planned successfully. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } else if (fraction > 0.3) {
    RCLCPP_WARN(node->get_logger(), "Path %.2f%% planned. Executing partial path...", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Path planning failed. Only %.2f%% completed.", fraction * 100.0);
    RCLCPP_ERROR(node->get_logger(), "Suggestions:");
    RCLCPP_ERROR(node->get_logger(), "  - Waypoints may be out of reach, try smaller rectangle");
    RCLCPP_ERROR(node->get_logger(), "  - Increase z_height to give more freedom");
    RCLCPP_ERROR(node->get_logger(), "  - Check for collisions in RViz");
  }

  // Spawn cubes in Gazebo Sim
  std::vector<Cube> cubes;
  int count = 0;
  for (const auto& pos : positions) {
    double x = std::get<0>(pos);
    double y = std::get<1>(pos);
    double h = max_height;

    auto it = selected_positions.find({x, y});
    if (it != selected_positions.end()) {
      h = it->second;
    }

    geometry_msgs::msg::Pose pose_cube;
    pose_cube.position.x = x;
    pose_cube.position.y = y;
    pose_cube.position.z = z_base + h / 2.0;
    pose_cube.orientation.x = 0.0;
    pose_cube.orientation.y = 0.0;
    pose_cube.orientation.z = 0.0;
    pose_cube.orientation.w = 1.0;

    cubes.push_back(Cube{pose_cube, h, count++});
  }

  std::string sdf_all = generate_multi_box_sdf(cubes, std::abs(cube_size_x), std::abs(cube_size_y));
  std::string path = "/tmp/multi_cubes.sdf";
  std::ofstream out(path);
  out << sdf_all;
  out.close();

  RCLCPP_INFO(node->get_logger(), "Spawning %d cubes in Gazebo Sim...", count);

  // Use gz service to spawn in Gazebo Sim (Ignition), not gazebo_ros
  std::ostringstream cmd;
  cmd << "gz service -s /world/default/create "
      << "--reqtype gz.msgs.EntityFactory "
      << "--reptype gz.msgs.Boolean "
      << "--timeout 5000 "
      << "--req 'sdf_filename: \"" << path << "\", name: \"spray_cubes\"'";
  int spawn_result = std::system(cmd.str().c_str());
  if (spawn_result != 0) {
    RCLCPP_WARN(node->get_logger(), "gz service spawn failed (code %d). Trying alternative...", spawn_result);
    // Fallback: try ros2 run ros_gz_sim create
    std::ostringstream cmd2;
    cmd2 << "ros2 run ros_gz_sim create -file " << path << " -name spray_cubes 2>&1";
    std::system(cmd2.str().c_str());
  }

  // Wait for user command to move to selected positions
  std::string go_cmd;
  std::cout << "\nType 'go' to move the robot to selected positions:\n";
  std::getline(std::cin, go_cmd);
  if (go_cmd == "go") {
    std::vector<geometry_msgs::msg::Pose> selected_waypoints;
    for (const auto& sel : selected_positions) {
      geometry_msgs::msg::Pose p;
      p.position.x = sel.first.first - robot_base_x;
      p.position.y = sel.first.second - robot_base_y;
      p.position.z = z_height;
      p.orientation = orientation;
      selected_waypoints.push_back(p);
    }

    std::sort(selected_waypoints.begin(), selected_waypoints.end(), [](const auto& a, const auto& b) {
      return a.position.x < b.position.x;
    });

    moveit_msgs::msg::RobotTrajectory traj2;
    double frac2 = move_group.computeCartesianPath(selected_waypoints, eef_step, jump_threshold, traj2);
    if (frac2 > 0.9) {
      RCLCPP_INFO(rclcpp::get_logger("planner"), "Final path %.2f%% planned successfully.", frac2 * 100.0);
      move_group.execute(traj2);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("planner"), "Final path planning failed. Only %.2f%% completed.", frac2 * 100.0);
    }
  }

  rclcpp::shutdown();
  return 0;
}
