#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <thread>
#include <atomic>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class LidarSurfaceScanner : public rclcpp::Node {
public:
  LidarSurfaceScanner()
  : Node("lidar_surface_scanner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), collecting_(false) {
    RCLCPP_INFO(this->get_logger(), "Initializing LIDAR Surface Scanner...");

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/gazebo_ros_laser/out", 10,
      std::bind(&LidarSurfaceScanner::cloudCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
      });

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&LidarSurfaceScanner::publishCloud, this));

    waitForTF("base_link", "lidar_link");

    std::thread([this]() {
      this->executeTrajectory();
      clipPointCloud();
      RCLCPP_INFO(this->get_logger(), "First Trajectory complete. Stopped collecting.");
    }).detach();
  }

private:
  void waitForTF(const std::string& target, const std::string& source, int max_attempts = 50) {
    for (int i = 0; i < max_attempts; ++i) {
      try {
        tf_buffer_.lookupTransform(target, source, tf2::TimePointZero, tf2::durationFromSec(1.0));
        RCLCPP_INFO(this->get_logger(), "TF from '%s' to '%s' is available.", source.c_str(), target.c_str());
        return;
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Waiting for TF from '%s' to '%s'... (%d/%d)", source.c_str(), target.c_str(), i + 1, max_attempts);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
    RCLCPP_ERROR(this->get_logger(), "TF not available. Exiting...");
    rclcpp::shutdown();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!collecting_) return;
    try {
      auto transform = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*msg, transformed, transform);
      PointCloud pcl_cloud;
      pcl::fromROSMsg(transformed, pcl_cloud);
      if (!pcl_cloud.empty()) accumulated_cloud_ += pcl_cloud;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", e.what());
    }
  }

  void publishCloud() {
    if (accumulated_cloud_.empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(accumulated_cloud_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    cloud_pub_->publish(msg);
  }

  void moveWithWaypoints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                         const std::vector<geometry_msgs::msg::Pose>& waypoints, bool collect) {
    if (waypoints.empty()) return;
    collecting_ = collect;
    moveit_msgs::msg::RobotTrajectory traj;
    double eef_step = 0.01, jump_thresh = 0.0;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_thresh, traj);
    if (fraction > 0.9) {
      RCLCPP_INFO(this->get_logger(), "Planned segment with %.1f%% success.", fraction * 100.0);
      double velocity_scaling = 0.025;
      double time_scaling = 1.0 / velocity_scaling;
      for (auto& point : traj.joint_trajectory.points) {
        double t = static_cast<double>(point.time_from_start.sec) +
                   static_cast<double>(point.time_from_start.nanosec) / 1e9;
        double scaled = t * time_scaling;
        point.time_from_start.sec = static_cast<int32_t>(scaled);
        point.time_from_start.nanosec = static_cast<uint32_t>((scaled - point.time_from_start.sec) * 1e9);
        for (auto& v : point.velocities) v /= time_scaling;
        for (auto& a : point.accelerations) a /= (time_scaling * time_scaling);
      }
      move_group->execute(traj);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Segment planning failed (%.1f%% success)", fraction * 100.0);
    }
    collecting_ = false;
  }

  void executeTrajectory() {
    const double robot_base_x = 0.25;
    std::vector<std::pair<double, double>> corners = {
      {0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4}
    };
    double mid_y = (corners[0].second + corners[3].second) / 2.0;
    double mid_x = (corners[0].first + corners[2].first) / 2.0;
    double x_start = corners[0].first;
    double x_end = corners[2].first;
    double y_start = corners[0].second;
    double y_end = corners[3].second;
    double z_fixed = 0.4;

    auto makePose = [](double x, double y, double z, double or_x, double or_y, double or_z, double or_w) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation.x = or_x;
      pose.orientation.y = or_y;
      pose.orientation.z = or_z;
      pose.orientation.w = or_w;
      return pose;
    };

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group->setPlanningTime(50.0);
    move_group->setMaxVelocityScalingFactor(0.2);
    move_group->setMaxAccelerationScalingFactor(0.5);

    moveWithWaypoints(move_group, {makePose(x_start - robot_base_x, mid_y, z_fixed, 0, 1, 0, 0)}, false);
    moveWithWaypoints(move_group, {makePose(x_end - robot_base_x, mid_y, z_fixed, 0, 1, 0, 0)}, true);

    std::vector<double> joint_position_1 = {
      -14.0 * M_PI / 180.0,   // shoulder_pan_joint
      -80.0 * M_PI / 180.0,  // shoulder_lift_joint
      108.0 * M_PI / 180.0,  // elbow_joint
      -118.0 * M_PI / 180.0, // wrist_1_joint
      -90.0 * M_PI / 180.0,  // wrist_2_joint
      -14.0 * M_PI / 180.0   // wrist_3_joint
    };

    std::vector<double> joint_position_2 = {
      16.0 * M_PI / 180.0,   // shoulder_pan_joint
      -71.0 * M_PI / 180.0,  // shoulder_lift_joint
      97.0 * M_PI / 180.0,   // elbow_joint
      -115.0 * M_PI / 180.0, // wrist_1_joint
      -90.0 * M_PI / 180.0,  // wrist_2_joint
      16.0 * M_PI / 180.0    // wrist_3_joint
    };

    // Move to first joint position
    move_group->setJointValueTarget(joint_position_1);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (move_group->plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group->execute(plan1);
      RCLCPP_INFO(this->get_logger(), "Moved to first joint position.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan first joint position.");
    }

    // Start collecting before second pose
    collecting_ = true;

    // Move to second joint position
    move_group->setJointValueTarget(joint_position_2);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    if (move_group->plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      
      // Scale down velocity and acceleration
      double velocity_scaling = 0.1;
      double time_scaling = 1.0 / velocity_scaling;

      for (auto& point : plan2.trajectory_.joint_trajectory.points) {
        double t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
        double scaled = t * time_scaling;
        point.time_from_start.sec = static_cast<int32_t>(scaled);
        point.time_from_start.nanosec = static_cast<uint32_t>((scaled - point.time_from_start.sec) * 1e9);

        for (auto& v : point.velocities) v *= velocity_scaling;
        for (auto& a : point.accelerations) a *= velocity_scaling * velocity_scaling;
      }

      move_group->execute(plan2);
      RCLCPP_INFO(this->get_logger(), "Moved to second joint position (with velocity scaling).");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan second joint position.");
    }

    // Stop collecting
    collecting_ = false;
   

    RCLCPP_INFO(this->get_logger(), "All trajectories complete.");
  }

  void clipPointCloud() {
    PointCloud clipped;
    for (const auto& pt : accumulated_cloud_.points) {
      if (pt.x >= 0.55 && pt.x <= 0.95 && pt.y >= 0.0 && pt.y <= 0.4) {
        clipped.points.push_back(pt);
      }
    }
    accumulated_cloud_ = clipped;
    RCLCPP_INFO(this->get_logger(), "Clipped point cloud to %lu points.", accumulated_cloud_.size());
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> collecting_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_received_ = false;
  PointCloud accumulated_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSurfaceScanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}