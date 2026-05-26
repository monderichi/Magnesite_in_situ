#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <numeric>
#include <limits>
#include <Eigen/Dense>
#include <algorithm>
#include <map>
#include <fstream>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <sstream>
#include <cstdlib>
#include <ctime>

#include <set>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_eigen/tf2_eigen.h>
#include "rclcpp/parameter_client.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = rclcpp::Node::make_shared("spray_sim_node", options);

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() {
      executor->spin();
  });

  // === Load kinematics.yaml and set parameters ===
  std::string kinematics_path = ament_index_cpp::get_package_share_directory("ur_moveit_config") + "/config/kinematics.yaml";
  YAML::Node yaml = YAML::LoadFile(kinematics_path);

  if (yaml["/**"] && yaml["/**"]["ros__parameters"] && yaml["/**"]["ros__parameters"]["robot_description_kinematics"]) {
    auto kinematics = yaml["/**"]["ros__parameters"]["robot_description_kinematics"];
    for (auto group : kinematics) {
      std::string group_name = group.first.as<std::string>();
      for (auto param : group.second) {
        std::string param_name = "robot_description_kinematics." + group_name + "." + param.first.as<std::string>();
        auto value = param.second;
        if (value.IsScalar()) {
          if (value.Tag() == "!!str" || value.IsSequence()) {
            node->declare_parameter(param_name, value.as<std::string>());
          } else if (value.IsScalar()) {
            try {
              node->declare_parameter(param_name, value.as<double>());
            } catch (...) {
              try {
                node->declare_parameter(param_name, value.as<int>());
              } catch (...) {
                node->declare_parameter(param_name, value.as<std::string>());
              }
            }
          }
        }
      }
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  geometry_msgs::msg::Pose target_pose;
  //target_pose.position.x = 0.561;
  target_pose.position.x = 1.1;
  target_pose.position.y = 0.350;
  target_pose.position.z = 0.2;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 1.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  // === Set pose target ===
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Planning to pose target succeeded.");

    // === Print IK solution ===
    const std::string planning_group = move_group.getName();
    auto robot_state = move_group.getCurrentState(10.0);  // wait up to 10 sec for state
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(planning_group, joint_values);
    const auto& joint_names = robot_state->getJointModelGroup(planning_group)->getVariableNames();

    RCLCPP_INFO(node->get_logger(), "IK Solution from PoseTarget:");
    for (size_t i = 0; i < joint_names.size(); ++i) {
      double deg = joint_values[i] * 180.0 / M_PI;
      RCLCPP_INFO(node->get_logger(), "  %s = %.6f rad = %.2f°", joint_names[i].c_str(), joint_values[i], deg);
    }

    // === Execute ===
    move_group.execute(my_plan);

    rclcpp::sleep_for(std::chrono::milliseconds(200));
    // === Get and print final end-effector pose ===
    geometry_msgs::msg::Pose final_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "Final End-Effector Position:");
    RCLCPP_INFO(node->get_logger(), "  x = %.6f", final_pose.position.x);
    RCLCPP_INFO(node->get_logger(), "  y = %.6f", final_pose.position.y);
    RCLCPP_INFO(node->get_logger(), "  z = %.6f", final_pose.position.z);

    RCLCPP_INFO(node->get_logger(), "Final End-Effector Orientation:");
    RCLCPP_INFO(node->get_logger(), "  x = %.6f", final_pose.orientation.x);
    RCLCPP_INFO(node->get_logger(), "  y = %.6f", final_pose.orientation.y);
    RCLCPP_INFO(node->get_logger(), "  z = %.6f", final_pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "  w = %.6f", final_pose.orientation.w);

  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning to pose target failed.");
  }

  executor->cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;

}
