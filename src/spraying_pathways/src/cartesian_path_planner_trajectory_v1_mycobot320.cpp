#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

using Point2D = std::pair<double, double>;

// Sorts 4 corner points to: top-left, top-right, bottom-right, bottom-left
std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& points) {
  if (points.size() != 4)
    throw std::runtime_error("Exactly 4 points are required.");

  auto sorted = points;
  std::sort(sorted.begin(), sorted.end(), [](const Point2D& a, const Point2D& b) {
    return a.second > b.second || (std::abs(a.second - b.second) < 0.001 && a.first < b.first);
  });

  std::vector<Point2D> top(sorted.begin(), sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin() + 2, sorted.end());

  std::sort(top.begin(), top.end());
  std::sort(bottom.begin(), bottom.end());

  return {top[0], top[1], bottom[1], bottom[0]};
}

// Generate zig-zag spray waypoints
std::vector<geometry_msgs::msg::Pose> generate_spray_waypoints(
    const Point2D& p1, const Point2D& p2, const Point2D& p4,
    double spray_width, double height_z,
    const geometry_msgs::msg::Quaternion& orientation)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto dx = p2.first - p1.first;
  auto dy = p4.second - p1.second;

  double length = std::sqrt(dx*dx + (p2.second - p1.second)*(p2.second - p1.second));
  double height = std::sqrt((p4.first - p1.first)*(p4.first - p1.first) + dy*dy);

  int cols = std::max(1, static_cast<int>(std::floor(length / spray_width)));
  int rows = std::max(1, static_cast<int>(std::floor(height / spray_width)));

  double step_x = dx / cols;
  double step_y = dy / rows;

  RCLCPP_INFO(rclcpp::get_logger("planner"), "Spray grid: %d rows x %d columns", rows, cols);
  RCLCPP_INFO(rclcpp::get_logger("planner"), "Step X: %.3f, Step Y: %.3f", step_x, step_y);

  int count = 0;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int col = (i % 2 == 0) ? j : (cols - j - 1);

      geometry_msgs::msg::Pose pose;
      pose.position.x = p1.first + (col + 0.5) * step_x;
      pose.position.y = p1.second + (i + 0.5) * step_y;
      pose.position.z = height_z;
      pose.orientation = orientation;

      waypoints.push_back(pose);

      std::stringstream ss;
      ss << std::fixed << std::setprecision(3);
      ss << "Point [" << count << "] -> x: " << pose.position.x
         << ", y: " << pose.position.y << ", z: " << pose.position.z;
      RCLCPP_INFO(rclcpp::get_logger("planner"), "%s", ss.str().c_str());
      count++;
    }
  }

  return waypoints;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_path_planner_mycobot320");

  RCLCPP_INFO(node->get_logger(), "============================================");
  RCLCPP_INFO(node->get_logger(), "myCobot 320 Cartesian Path Planner");
  RCLCPP_INFO(node->get_logger(), "============================================");

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  
  // Get current pose first
  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Move to a known good "ready" position first
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
    RCLCPP_INFO(node->get_logger(), "At ready position, proceeding to spray pattern...");
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not plan to 'ready' position, trying from current position...");
  }

  // Now set parameters for Cartesian path
  move_group.setPlanningTime(30.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Define rectangle corners - MUCH CLOSER TO BASE for myCobot 320
  // myCobot 320 has ~350mm reach, so we work in a smaller area
  std::vector<Point2D> unordered_points = {
    {0.25, -0.10},   // front-left (closer X)
    {0.25, 0.10},    // front-right  
    {0.40, -0.10},   // back-left
    {0.40, 0.10}     // back-right
  };

  auto corners = sort_rectangle_corners(unordered_points);
  RCLCPP_INFO(node->get_logger(), "Rectangle corners (TL, TR, BR, BL):");
  RCLCPP_INFO(node->get_logger(), "  TL: (%.3f, %.3f)", corners[0].first, corners[0].second);
  RCLCPP_INFO(node->get_logger(), "  TR: (%.3f, %.3f)", corners[1].first, corners[1].second);
  RCLCPP_INFO(node->get_logger(), "  BR: (%.3f, %.3f)", corners[2].first, corners[2].second);
  RCLCPP_INFO(node->get_logger(), "  BL: (%.3f, %.3f)", corners[3].first, corners[3].second);

  // Spray parameters
  double spray_width = 0.03;    // 3cm spacing
  double z_height = 0.12;       // 12cm height (lower for myCobot)

  // Tool orientation - angled slightly forward instead of straight down
  // This helps avoid wrist singularities
  tf2::Quaternion quat;
  quat.setRPY(0.0, M_PI/3, 0.0);  // 60 degrees pitch (tilted forward)
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = quat.x();
  orientation.y = quat.y();
  orientation.z = quat.z();
  orientation.w = quat.w();

  RCLCPP_INFO(node->get_logger(), "Tool orientation: rpy=(0, 60, 0) degrees");

  // Generate spray path
  auto waypoints = generate_spray_waypoints(
    corners[0], corners[1], corners[3], spray_width, z_height, orientation
  );

  RCLCPP_INFO(node->get_logger(), "Generated %zu waypoints", waypoints.size());
  RCLCPP_INFO(node->get_logger(), "Target area: X=[%.2f, %.2f], Y=[%.2f, %.2f], Z=%.2f",
              corners[0].first, corners[1].first,
              corners[3].second, corners[0].second, z_height);

  // Test if first waypoint is reachable
  RCLCPP_INFO(node->get_logger(), "Testing first waypoint reachability...");
  move_group.setPoseTarget(waypoints[0]);
  moveit::planning_interface::MoveGroupInterface::Plan test_plan;
  bool test_success = (move_group.plan(test_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (test_success) {
    RCLCPP_INFO(node->get_logger(), "First waypoint is REACHABLE!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "First waypoint is NOT reachable! Aborting.");
    RCLCPP_ERROR(node->get_logger(), "Tip: The target may be too far or in collision.");
    rclcpp::shutdown();
    return 1;
  }

  // Plan Cartesian path
  RCLCPP_INFO(node->get_logger(), "Planning Cartesian path through all waypoints...");
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;  // Smaller step for better accuracy
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.8) {
    RCLCPP_INFO(node->get_logger(), "Path %.1f%% planned successfully. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(), "Motion complete!");
  } else if (fraction > 0.0) {
    RCLCPP_WARN(node->get_logger(), "Only %.1f%% path planned. Executing partial path...", fraction * 100.0);
    move_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(), "Partial motion complete!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Path planning failed. No valid path found.");
    RCLCPP_ERROR(node->get_logger(), "Suggestions:");
    RCLCPP_ERROR(node->get_logger(), "  - Move waypoints even closer to base");
    RCLCPP_ERROR(node->get_logger(), "  - Reduce Z height further");
    RCLCPP_ERROR(node->get_logger(), "  - Use different tool orientation");
    RCLCPP_ERROR(node->get_logger(), "  - Check for collisions in RViz");
  }

  rclcpp::shutdown();
  return 0;
}
