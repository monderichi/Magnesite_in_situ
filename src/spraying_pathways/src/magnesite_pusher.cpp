#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/float64.hpp>

class MagnesitePusher : public rclcpp::Node
{
public:
  MagnesitePusher() : Node("magnesite_pusher") {
    group_name_      = this->declare_parameter("group_name", "ur_manipulator");
    push_distance_   = this->declare_parameter("push_distance", 0.14);
    velocity_scale_  = this->declare_parameter("velocity_scale", 0.1);
    accel_scale_     = this->declare_parameter("accel_scale", 0.1);

    // Park / push-start joint values:
    // Robot is at conveyor level, tool 1cm above belt, ready to push along -X.
    // These are the STARTING position of the linear push.
    park_joints_[0]  = this->declare_parameter("park_j0", -0.061261056745000965);
    park_joints_[1]  = this->declare_parameter("park_j1",  1.85598312657077);
    park_joints_[2]  = this->declare_parameter("park_j2", -1.7671458676442586);
    park_joints_[3]  = this->declare_parameter("park_j3",  1.5446163880149817);
    park_joints_[4]  = this->declare_parameter("park_j4", -1.613731426393957);
    park_joints_[5]  = this->declare_parameter("park_j5", -0.1303760951239764);

    // Push-end joint values (fallback if Cartesian path planning fails):
    // These are the joints at the END of the 14cm -X push.
    push_end_joints_[0] = this->declare_parameter("push_end_j0", -0.07504915783575616);
    push_end_joints_[1] = this->declare_parameter("push_end_j1",  1.4203489452729854);
    push_end_joints_[2] = this->declare_parameter("push_end_j2", -0.4279547325890096);
    push_end_joints_[3] = this->declare_parameter("push_end_j3",  0.5122541354603357);
    push_end_joints_[4] = this->declare_parameter("push_end_j4", -1.556833692778942);
    push_end_joints_[5] = this->declare_parameter("push_end_j5", -0.15323990832510212);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/magnesite_target", 10,
      std::bind(&MagnesitePusher::targetCallback, this, std::placeholders::_1)
    );

    exec_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/magnesite_exec_time", 10);

    RCLCPP_INFO(this->get_logger(),
      "Pusher ready: vel=%.0f%%, push=%.0fcm. Linear push along -X from fixed park pose.",
      velocity_scale_ * 100.0, push_distance_ * 100.0);
  }

private:
  void ensureMoveGroup() {
    if (!move_group_) {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), group_name_);
      move_group_->setMaxVelocityScalingFactor(velocity_scale_);
      move_group_->setMaxAccelerationScalingFactor(accel_scale_);
      move_group_->setPlanningTime(5.0);
      move_group_->setNumPlanningAttempts(3);
      RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
      rclcpp::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(this->get_logger(), "MoveGroup ready.");
    }
  }

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (pushing_) {
      RCLCPP_WARN(this->get_logger(), "Already pushing, ignoring.");
      return;
    }
    pushing_ = true;
    auto t_start = this->now();
    ensureMoveGroup();

    try {
      RCLCPP_INFO(this->get_logger(), "Target received [%s]: X=%.3f, Y=%.3f, Z=%.3f (used as trigger only)",
        msg->header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z);

      // ---- STEP 1: Ensure we are at park (push-start) position ----
      // The robot should already be at park, but verify/move if needed.
      goPark();

      // ---- STEP 2: Get current EE pose (at park = push start) ----
      auto start_pose = move_group_->getCurrentPose().pose;
      RCLCPP_INFO(this->get_logger(),
        "Push start (tool0): X=%.3f, Y=%.3f, Z=%.3f",
        start_pose.position.x, start_pose.position.y, start_pose.position.z);

      // ---- STEP 3: Compute end pose: push along -X, keep Y/Z/orientation ----
      geometry_msgs::msg::Pose end_pose = start_pose;
      end_pose.position.x = start_pose.position.x - push_distance_;

      RCLCPP_INFO(this->get_logger(),
        "Linear push: X=%.3f -> X=%.3f (%.1f cm along -X), fixed Y=%.3f Z=%.3f",
        start_pose.position.x, end_pose.position.x,
        push_distance_ * 100.0,
        start_pose.position.y, start_pose.position.z);

      // ---- STEP 4: Cartesian linear path from start to end ----
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(end_pose);

      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group_->computeCartesianPath(
        waypoints, 0.005, 0.0, trajectory, true);

      RCLCPP_INFO(this->get_logger(),
        "Cartesian path fraction: %.0f%%", fraction * 100.0);

      if (fraction > 0.5) {
        // Good enough path: execute the Cartesian trajectory
        RCLCPP_INFO(this->get_logger(),
          "Executing Cartesian push (%.0f%% of path)...", fraction * 100.0);
        auto exec_result = move_group_->execute(trajectory);
        if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Cartesian push executed successfully!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Cartesian push execution failed, trying joint-space fallback.");
          pushViaJoints();
        }
      } else {
        // Cartesian planning failed: fall back to joint-space push using known end joints
        RCLCPP_WARN(this->get_logger(),
          "Cartesian path only %.0f%% achieved. Using joint-space fallback push.", fraction * 100.0);
        pushViaJoints();
      }

      // ---- STEP 5: Return to park (push-start position) ----
      goPark();

      // Publish measured execution time for rendezvous solver feedback
      auto t_end = this->now();
      double elapsed = (t_end - t_start).seconds();
      std_msgs::msg::Float64 et_msg;
      et_msg.data = elapsed;
      exec_time_pub_->publish(et_msg);
      RCLCPP_INFO(this->get_logger(), "Measured cycle time: %.2f s", elapsed);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF failed: %s", ex.what());
    } catch (std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", ex.what());
    }
    pushing_ = false;
  }

  void goPark() {
    RCLCPP_INFO(this->get_logger(), "Moving to park (push-start) position...");
    std::vector<double> joint_vals(park_joints_.begin(), park_joints_.end());
    move_group_->setJointValueTarget(joint_vals);
    auto result = move_group_->move();
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Park move failed.");
    } else {
      RCLCPP_INFO(this->get_logger(), "At park position, ready.");
    }
  }

  void pushViaJoints() {
    // Fallback: joint-space move to the known push-end position.
    // Less precise (not guaranteed linear) but ensures the push happens.
    RCLCPP_INFO(this->get_logger(), "Joint-space fallback push to end position...");
    std::vector<double> end_vals(push_end_joints_.begin(), push_end_joints_.end());
    move_group_->setJointValueTarget(end_vals);
    auto result = move_group_->move();
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Joint-space push also failed!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Joint-space push done.");
    }
  }

  std::string group_name_;
  double push_distance_;
  double velocity_scale_, accel_scale_;
  std::array<double, 6> park_joints_;
  std::array<double, 6> push_end_joints_;
  bool pushing_ = false;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr exec_time_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MagnesitePusher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
