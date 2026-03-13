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
    sdf << "        <script>\n";
    sdf << "          <uri>file:///ros2_ws/src/spraying_pathways/materials/scripts</uri>\n";
    sdf << "          <name>My/Seaweed</name>\n";
    sdf << "        </script>\n";
    sdf << "        <ambient>1 1 1 0.7</ambient>\n";   // RGBA - 0.7 alpha = 70% visible
    sdf << "        <diffuse>1 1 1 0.7</diffuse>\n";   // Controls lighting and transparency
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
  auto node = rclcpp::Node::make_shared("cartesian_path_planner");
  std::srand(std::time(nullptr));

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  std::vector<Point2D> unordered_points = {
    {0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4}
  };
  auto corners = sort_rectangle_corners(unordered_points);

  double spray_width = 0.04;
  double z_height = 0.4;
  double max_height = 0.02;
  double z_base = 0.715 + 0.05;  // updated

  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;

  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f corners[0].first , %f corners[0].second", corners[0].first, corners[0].second);
  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f corners[1].first , %f corners[1].second", corners[1].first, corners[1].second);
  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f corners[2].first , %f corners[2].second", corners[2].first, corners[2].second);
  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f corners[3].first , %f corners[3].second", corners[3].first, corners[3].second);

  auto dx = corners[1].first - corners[0].first;
  auto dy = corners[3].second - corners[0].second;
  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f dx , %f dy", dx, dy);

  double length = std::hypot(dx, corners[1].second - corners[0].second);
  double height = std::hypot(corners[3].first - corners[0].first, dy);

  //RCLCPP_INFO(rclcpp::get_logger("planner"), "%f length , %f height", length, height);

  int cols = std::max(1, static_cast<int>(std::floor(length / spray_width)));
  int rows = std::max(1, static_cast<int>(std::floor(height / spray_width)));

  double step_x = dx / cols;
  double step_y = dy / rows;

  RCLCPP_INFO(rclcpp::get_logger("planner"), "Spray grid: %d rows x %d columns", rows, cols);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<Cube> cubes;
  std::vector<std::pair<double, double>> positions;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int col = (i % 2 == 0) ? j : (cols - j - 1);

      geometry_msgs::msg::Pose pose;
      pose.position.x = corners[0].first + (col + 0.5) * step_x - robot_base_x;
      pose.position.y = corners[0].second + (i + 0.5) * step_y - robot_base_y;
      pose.position.z = z_height;
      pose.orientation = orientation;

      waypoints.push_back(pose);

      double x = round_to(pose.position.x + robot_base_x, 4);  // rebase to world
      double y = round_to(pose.position.y + robot_base_y, 4);
      positions.push_back({x, y});
    }
  }

  std::map<std::pair<double, double>, double> selected_positions;
  std::cout << "Available positions (x y):\n";
  for (const auto& [x, y] : positions) {
    std::cout << std::fixed << std::setprecision(4) << x << " " << y << "\n";
  }

  std::cout << "Enter positions and height (x y h), type 'done' to finish:\n";
  std::string input;
  while (true) {
    std::getline(std::cin, input);
    if (input == "done") break;
    std::istringstream iss(input);
    double x, y, h;
    if (iss >> x >> y >> h) {
      x = round_to(x, 4);
      y = round_to(y, 4);
      selected_positions[{x, y}] = h;
    }
  }

  int count = 0;
  for (const auto& [x, y] : positions) {
    double final_height = max_height;
    auto it = selected_positions.find({x, y});
    if (it != selected_positions.end()) {
      final_height = it->second;
    }
    geometry_msgs::msg::Pose pose_cube;
    pose_cube.position.x = x;
    pose_cube.position.y = y;
    pose_cube.position.z = z_base + final_height / 2.0;
    pose_cube.orientation = orientation;
    cubes.push_back(Cube{pose_cube, final_height, count++});
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.9) {
    RCLCPP_INFO(node->get_logger(), "Path %.2f%% planned successfully. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Path planning failed. Only %.2f%% completed.", fraction * 100.0);
  }

  std::string sdf_all = generate_multi_box_sdf(cubes, std::abs(step_x), std::abs(step_y));
  std::string path = "/tmp/multi_cubes.sdf";
  std::ofstream out(path);
  out << sdf_all;
  out.close();

  std::ostringstream cmd;
  cmd << "ros2 run gazebo_ros spawn_entity.py"
      << " -file " << path
      << " -entity all_cubes";
  std::system(cmd.str().c_str());

  rclcpp::shutdown();
  return 0;
}