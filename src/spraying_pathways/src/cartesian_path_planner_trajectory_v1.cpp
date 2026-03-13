#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>  // for std::setprecision

using Point2D = std::pair<double, double>;

// Sorts 4 corner points to: top-left, top-right, bottom-right, bottom-left
std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& points) {
  if (points.size() != 4)
    throw std::runtime_error("Exactly 4 points are required.");

  auto sorted = points;

  // Sort by Y descending, then X ascending
  std::sort(sorted.begin(), sorted.end(), [](const Point2D& a, const Point2D& b) {
    return a.second > b.second || (a.second == b.second && a.first < b.first);
  });

  std::vector<Point2D> top(sorted.begin(), sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin() + 2, sorted.end());

  std::sort(top.begin(), top.end());      // TL, TR
  std::sort(bottom.begin(), bottom.end()); // BL, BR

  return {top[0], top[1], bottom[1], bottom[0]}; // TL, TR, BR, BL
}

// Generate zig-zag spray waypoints
std::vector<geometry_msgs::msg::Pose> generate_spray_waypoints(
    const Point2D& p1, const Point2D& p2,
    const Point2D& p4,
    double spray_width, double height_z,
    const geometry_msgs::msg::Quaternion& orientation)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Robot base offset (NEW)
  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  auto dx = p2.first - p1.first;
  auto dy = p4.second - p1.second;

  double length = std::sqrt((p2.first - p1.first)*(p2.first - p1.first) + (p2.second - p1.second)*(p2.second - p1.second));
  double height = std::sqrt((p4.first - p1.first)*(p4.first - p1.first) + (p4.second - p1.second)*(p4.second - p1.second));

  int cols = std::max(1, static_cast<int>(std::floor(length / spray_width)));
  int rows = std::max(1, static_cast<int>(std::floor(height / spray_width)));

  double step_x = dx / cols;
  double step_y = dy / rows;

  RCLCPP_INFO(rclcpp::get_logger("planner"), "Spray grid: %d rows x %d columns", rows, cols);

  int count = 0;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int col = (i % 2 == 0) ? j : (cols - j - 1); // zig-zag

      geometry_msgs::msg::Pose pose;
      pose.position.x = p1.first + (col + 0.5) * step_x - robot_base_x;
      pose.position.y = p1.second + (i + 0.5) * step_y - robot_base_y;
      pose.position.z = height_z;
      pose.orientation = orientation;

      waypoints.push_back(pose);

      // Log each point with index
      std::stringstream ss;
      ss << std::fixed << std::setprecision(3);
      ss << "Point [" << count << "] → x: " << pose.position.x
         << ", y: " << pose.position.y
         << ", z: " << pose.position.z;
      RCLCPP_INFO(rclcpp::get_logger("planner"), ss.str().c_str());
      count++;
    }
  }

  return waypoints;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_path_planner");

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Define 4 unordered corner points (x, y)
  std::vector<Point2D> unordered_points = {
    {0.8, 0.0},
    {0.8, 0.4},
    {1.2, 0.0},
    {1.2, 0.4}
  };

  auto corners = sort_rectangle_corners(unordered_points); // TL, TR, BR, BL

  // Spray parameters
  double spray_width = 0.02; // distance between spray lines
  double z_height = 0.4;    // fixed spray height
  double z_base = 0.715 + 0.05; // defined for consistency (not used directly here)

  // Tool orientation (pointing downward -Z)
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;

  // Generate spray path
  auto waypoints = generate_spray_waypoints(
    corners[0], corners[1], corners[3], spray_width, z_height, orientation
  );

  // Plan Cartesian path
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

  rclcpp::shutdown();
  return 0;
}
