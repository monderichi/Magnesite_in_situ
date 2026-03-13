#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using FollowJT = control_msgs::action::FollowJointTrajectory;

class SafetyStopOnUnknown : public rclcpp::Node
{
public:
  SafetyStopOnUnknown()
  : rclcpp::Node("safety_stop_on_unknown")
  {
    // Parameters (declare + get)
    group_name_      = declare_parameter<std::string>("group_name", "ur_manipulator");
    controller_ns_   = declare_parameter<std::string>("controller_ns", "joint_trajectory_controller");
    unknown_topic_   = declare_parameter<std::string>("unknown_topic", "/unknown_points");
    min_points_      = declare_parameter<int>("min_points", 1);              // πόσα άγνωστα για να θεωρηθεί «εμπόδιο»
    debounce_ms_     = declare_parameter<int>("debounce_ms", 500);           // πόσο να περιμένει μεταξύ stop
    wait_action_ms_  = declare_parameter<int>("wait_action_ms", 200);        // αναμονή για action server

    // Subscriber
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      unknown_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SafetyStopOnUnknown::unknownCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "SafetyStopOnUnknown up. group='%s', controller='%s', topic='%s'",
                group_name_.c_str(), controller_ns_.c_str(), unknown_topic_.c_str());
  }

private:
  // Lazy init για MoveGroupInterface (πρέπει να καλέσουμε shared_from_this() εκτός ctor)
  void ensureMoveGroup()
  {
    if (!mgi_) {
      mgi_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), group_name_);
      // συντηρητικά χαμηλό scaling για ασφαλέστερα stop/resume patterns
      mgi_->setMaxVelocityScalingFactor(0.2);
      mgi_->setMaxAccelerationScalingFactor(0.2);
      RCLCPP_INFO(get_logger(), "MoveGroupInterface ready for group '%s'", group_name_.c_str());
    }
  }

  // Lazy init action client προς τον controller
  void ensureTrajClient()
  {
    if (!traj_client_) {
      const std::string action_name = "/" + controller_ns_ + "/follow_joint_trajectory";
      traj_client_ = rclcpp_action::create_client<FollowJT>(shared_from_this(), action_name);
      RCLCPP_INFO(get_logger(), "Action client to '%s' created", action_name.c_str());
    }
  }

  void doEmergencyStop()
  {
    const auto now = this->get_clock()->now();
    if ((now - last_stop_time_) < rclcpp::Duration::from_seconds(debounce_ms_ / 1000.0))
      return;  // debounce

    ensureMoveGroup();
    ensureTrajClient();

    // 1) Προσπάθησε να ακυρώσεις ΟΛΟΥΣ τους στόχους στον controller (χαμηλού επιπέδου)
    if (traj_client_->wait_for_action_server(std::chrono::milliseconds(wait_action_ms_))) {
      (void)traj_client_->async_cancel_all_goals();
      RCLCPP_WARN(get_logger(), "[STOP] Sent async_cancel_all_goals() to FollowJointTrajectory");
    } else {
      RCLCPP_WARN(get_logger(), "[STOP] Action server not ready (will still call MoveIt stop())");
    }

    // 2) Ζήτα από MoveIt να σταματήσει ό,τι εκτελείται
    try {
      mgi_->stop();  // «ήπιο» στοπ μέσω MoveIt
      RCLCPP_ERROR(get_logger(), "[STOP] MoveIt stop() requested due to unknown points");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MoveIt stop() threw: %s", e.what());
    }

    last_stop_time_ = now;
  }

  void unknownCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Απλός έλεγχος πλήθους: width*height ~ σημεία (μην κάνουμε parse τα πεδία)
    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

    // Αν υπάρχουν αρκετά άγνωστα → STOP
    if (n >= static_cast<size_t>(min_points_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unknown cloud detected: %zu points -> STOP", n);
      doEmergencyStop();
    }
  }

private:
  // Params
  std::string group_name_;
  std::string controller_ns_;
  std::string unknown_topic_;
  int min_points_;
  int debounce_ms_;
  int wait_action_ms_;

  // MoveIt + Action client
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  rclcpp_action::Client<FollowJT>::SharedPtr traj_client_;

  // ROS bits
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  // State
  rclcpp::Time last_stop_time_{0,0, get_clock() ? get_clock()->get_clock_type() : RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyStopOnUnknown>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
