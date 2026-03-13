#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("follow_trajectory_node");

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

  std::vector<double> point1 = {1.57, -1.57, 1.57, 0.0, 1.57, 0.0};
  std::vector<double> point2 = {-1.5, -1.57, 1.8, 0.0, 1.57, 0.2};
  std::vector<double> point3 = {0.0, 1.2, 1.57, 0.0, 0.0, -0,2};

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningTime(5.0);

  RCLCPP_INFO(node->get_logger(), "Moving to start point...");
  move_group.setJointValueTarget(point1);
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  if (move_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan1);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning to start point failed.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Moving to second point...");
  move_group.setJointValueTarget(point2);
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  if (move_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan2);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning to second point failed.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Moving to third point...");
  move_group.setJointValueTarget(point3);
  moveit::planning_interface::MoveGroupInterface::Plan plan3;
  if (move_group.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan3);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning to third point failed.");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}