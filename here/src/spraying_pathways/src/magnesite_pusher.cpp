#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// FIX: added explicit headers for RobotState and RobotModel, which are used
// directly but were previously relying on fragile transitive includes.
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <std_msgs/msg/float64.hpp>
// FIX: added explicit Eigen header; previously relied on transitive MoveIt include.
#include <Eigen/Geometry>

class MagnesitePusher : public rclcpp::Node
{
public:
  MagnesitePusher() : Node("magnesite_pusher") {
    group_name_      = this->declare_parameter("group_name", "ur_manipulator");
    push_distance_   = this->declare_parameter("push_distance", 0.14);
    velocity_scale_  = this->declare_parameter("velocity_scale", 0.4);
    accel_scale_     = this->declare_parameter("accel_scale", 0.4);

    // Push-start joint values:
    // Robot is at conveyor level, tool 1cm above belt, ready to push along -X.
    // These are the STARTING position of the linear push.
    park_joints_[0]  = this->declare_parameter("park_j0", -0.16859880574265224);
    park_joints_[1]  = this->declare_parameter("park_j1",  1.6397368322486727);
    park_joints_[2]  = this->declare_parameter("park_j2", -1.6397368322486727);
    park_joints_[3]  = this->declare_parameter("park_j3",  1.6643459747017926);
    park_joints_[4]  = this->declare_parameter("park_j4", -1.5538666330505517);
    park_joints_[5]  = this->declare_parameter("park_j5", -0.24068090385001803);

    // Push-end joint values (fallback if Cartesian path planning fails):
    // These are the joints at the END of the 14cm -X push.
    push_end_joints_[0] = this->declare_parameter("push_end_j0", -0.19320794819577228);
    push_end_joints_[1] = this->declare_parameter("push_end_j1",  1.6214108751027323);
    push_end_joints_[2] = this->declare_parameter("push_end_j2", -0.937241808320955);
    push_end_joints_[3] = this->declare_parameter("push_end_j3",  0.6595599243286571);
    push_end_joints_[4] = this->declare_parameter("push_end_j4", -1.4679964338524305);
    push_end_joints_[5] = this->declare_parameter("push_end_j5", -0.26371924997634316);

    // Monitor joint values:
    // Robot waits here between pushes (safe watching position above belt).
    // Robot returns HERE after every push cycle.
    monitor_joints_[0] = this->declare_parameter("monitor_j0", -0.08429940287132612);
    monitor_joints_[1] = this->declare_parameter("monitor_j1",  0.7700392659798981);
    monitor_joints_[2] = this->declare_parameter("monitor_j2", -0.12409290981679684);
    monitor_joints_[3] = this->declare_parameter("monitor_j3",  0.9110618695410401);
    monitor_joints_[4] = this->declare_parameter("monitor_j4", -1.5599752854325317);
    monitor_joints_[5] = this->declare_parameter("monitor_j5", -0.16406094968746698);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/magnesite_target", 10,
      std::bind(&MagnesitePusher::targetCallback, this, std::placeholders::_1)
    );

    exec_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/magnesite_exec_time", 10);

    // Publishes ONLY the goPushStart travel time (monitor → push-start).
    // The Python trigger uses this directly: fire when belt travel time to
    // Y_PARK equals this value → rock arrives exactly as push starts.
    goto_push_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/magnesite_goto_push_t", 10);

    // FIX: MoveGroupInterface requires shared_from_this(), which is only valid
    // after the constructor returns (the node must already be owned by a
    // shared_ptr). Initialising it inside the subscription callback caused a
    // 2-second blocking sleep on the executor thread, starving all other
    // callbacks. We use a one-shot wall timer instead: it fires on the first
    // executor spin, after the constructor has completed, so shared_from_this()
    // is safe and the sleep happens before any real work arrives.
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        init_timer_->cancel();   // fire only once
        initMoveGroup();
      });

    RCLCPP_INFO(this->get_logger(),
      "Pusher ready: vel=%.0f%%, push=%.0fcm. Linear push along -X from fixed park pose.",
      velocity_scale_ * 100.0, push_distance_ * 100.0);
  }

private:
  // FIX: MoveGroup initialisation extracted to its own method and called from
  // a one-shot timer (see constructor). This avoids blocking the executor
  // thread with rclcpp::sleep_for() inside a subscription callback.
  void initMoveGroup() {
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

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (pushing_) {
      RCLCPP_WARN(this->get_logger(), "Already pushing, ignoring.");
      return;
    }

    // Guard: if the one-shot timer hasn't fired yet MoveGroup is not ready.
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroup not ready yet, ignoring target.");
      return;
    }

    pushing_ = true;
    auto t_start = this->now();

    try {
      RCLCPP_INFO(this->get_logger(), "Target received [%s]: X=%.3f, Y=%.3f, Z=%.3f",
        msg->header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z);

      // ---- STEP 1: Move from monitor to push-start position ----
      // Measure this time precisely — Python uses it to trigger at the
      // correct belt position (fire when rock travel time = t_goPushStart).
      auto t_goto_start = this->now();
      goPushStart();
      auto t_goto_end = this->now();
      double t_goPushStart = (t_goto_end - t_goto_start).seconds();
      std_msgs::msg::Float64 goto_msg;
      goto_msg.data = t_goPushStart;
      goto_push_time_pub_->publish(goto_msg);
      RCLCPP_INFO(this->get_logger(), "goPushStart time: %.3f s", t_goPushStart);

      // ---- STEP 2: Compute start pose via FK from known park joints ----
      // getCurrentPose() returns stale (0,0,0) because the myCobot serial driver
      // stops publishing joint states during trajectory execution.
      // Solution: build a RobotState from our known park_joints_ and compute FK.
      moveit::core::RobotStatePtr fk_state(
        new moveit::core::RobotState(move_group_->getRobotModel()));
      const auto* jmg = fk_state->getJointModelGroup(group_name_);
      fk_state->setJointGroupPositions(
        jmg, std::vector<double>(park_joints_.begin(), park_joints_.end()));
      fk_state->update();
      move_group_->setStartState(*fk_state);  // tell MoveIt our exact start state

      const Eigen::Isometry3d& ee_tf =
        fk_state->getGlobalLinkTransform(move_group_->getEndEffectorLink());
      geometry_msgs::msg::Pose start_pose;
      start_pose.position.x = ee_tf.translation().x();
      start_pose.position.y = ee_tf.translation().y();
      start_pose.position.z = ee_tf.translation().z();
      Eigen::Quaterniond q(ee_tf.rotation());
      start_pose.orientation.x = q.x();
      start_pose.orientation.y = q.y();
      start_pose.orientation.z = q.z();
      start_pose.orientation.w = q.w();

      RCLCPP_INFO(this->get_logger(),
        "Push start (FK): X=%.3f, Y=%.3f, Z=%.3f",
        start_pose.position.x, start_pose.position.y, start_pose.position.z);

      // ---- STEP 3: Compute end pose via FK of push_end_joints_ ----
      moveit::core::RobotStatePtr fk_end_state(
        new moveit::core::RobotState(move_group_->getRobotModel()));
      fk_end_state->setJointGroupPositions(
        jmg, std::vector<double>(push_end_joints_.begin(), push_end_joints_.end()));
      fk_end_state->update();
      const Eigen::Isometry3d& ee_end_tf =
        fk_end_state->getGlobalLinkTransform(move_group_->getEndEffectorLink());
      geometry_msgs::msg::Pose end_pose = start_pose;
      end_pose.position.x = ee_end_tf.translation().x();
      end_pose.position.y = ee_end_tf.translation().y();
      end_pose.position.z = ee_end_tf.translation().z();
      double push_dist = std::abs(end_pose.position.x - start_pose.position.x);
      RCLCPP_INFO(this->get_logger(),
        "Linear push: X=%.3f -> X=%.3f (%.1f cm, from push_end_joints FK)",
        start_pose.position.x, end_pose.position.x, push_dist * 100.0);

      // ---- STEP 4: Cartesian linear path ----
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(end_pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group_->computeCartesianPath(
        waypoints, 0.005, 0.0, trajectory, true);
      RCLCPP_INFO(this->get_logger(), "Cartesian fraction: %.0f%%", fraction * 100.0);

      if (fraction >= 0.99) {
        // FIX: execute(moveit_msgs::msg::RobotTrajectory) is not available in
        // all MoveIt2 versions. Wrapping in a Plan is the portable approach.
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto res = move_group_->execute(plan);
        // FIX: use moveit_msgs::msg::MoveItErrorCodes::SUCCESS (val == 1) for
        // the comparison instead of moveit::planning_interface::MoveItErrorCode::SUCCESS,
        // which may not compile on newer MoveIt2 / ROS2 distros.
        if (res.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Cartesian push done.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Cartesian execute failed (code %d), falling back to joint-space.", res.val);
          pushViaJoints();
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Cartesian only %.0f%% (< 99%%), using joint-space fallback to reach end.", fraction * 100.0);
        pushViaJoints();
      }

      // 30 ms settle at push-end before returning
      rclcpp::sleep_for(std::chrono::milliseconds(30));

      // ---- STEP 5: Return to MONITOR (watching) position ----
      goMonitor();

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

  void goPushStart() {
    RCLCPP_INFO(this->get_logger(), "Moving to push-start (conveyor level) position...");
    std::vector<double> joint_vals(park_joints_.begin(), park_joints_.end());
    move_group_->setJointValueTarget(joint_vals);
    auto result = move_group_->move();
    // FIX: consistent use of moveit_msgs comparison throughout.
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Push-start move failed (code %d).", result.val);
    } else {
      RCLCPP_INFO(this->get_logger(), "At push-start position, ready.");
    }
  }

  void goMonitor() {
    RCLCPP_INFO(this->get_logger(), "Returning to monitor (watching) position...");
    std::vector<double> joint_vals(monitor_joints_.begin(), monitor_joints_.end());
    move_group_->setJointValueTarget(joint_vals);
    auto result = move_group_->move();
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Monitor move failed (code %d).", result.val);
    } else {
      RCLCPP_INFO(this->get_logger(), "At monitor position, watching conveyor.");
    }
  }

  void pushViaJoints() {
    // Fallback: joint-space move to the known push-end position.
    // Less precise (not guaranteed linear) but ensures the push happens.
    RCLCPP_INFO(this->get_logger(), "Joint-space fallback push to end position...");
    std::vector<double> end_vals(push_end_joints_.begin(), push_end_joints_.end());
    move_group_->setJointValueTarget(end_vals);
    auto result = move_group_->move();
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Joint-space push also failed (code %d)!", result.val);
    } else {
      RCLCPP_INFO(this->get_logger(), "Joint-space push done.");
    }
  }

  std::string group_name_;
  double push_distance_;
  double velocity_scale_, accel_scale_;
  std::array<double, 6> park_joints_;
  std::array<double, 6> push_end_joints_;
  std::array<double, 6> monitor_joints_;
  bool pushing_ = false;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr exec_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr goto_push_time_pub_;
  // FIX: one-shot timer handle needed to cancel itself after first fire.
  rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MagnesitePusher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}