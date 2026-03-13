#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <set>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <sstream>
#include <chrono>
#include <numeric>

using std::placeholders::_1;

class DepthDipDetector : public rclcpp::Node
{
public:
  // Default ctor
  DepthDipDetector()
  : rclcpp::Node("depth_dip_detector")
  {
    init_interfaces_();
  }

  // Ctor with NodeOptions (so we can pass use_sim_time etc.)
  explicit DepthDipDetector(const rclcpp::NodeOptions& options)
  : rclcpp::Node("depth_dip_detector", options)
  {
    init_interfaces_();
  }

  void attach_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi)
  {
    move_group_ = std::move(mgi);
    RCLCPP_INFO(this->get_logger(), "MoveIt MoveGroup attached: '%s'", move_group_->getName().c_str());
  }

  void move_to_center_and_start()
  {
    RCLCPP_INFO(this->get_logger(), "Starting preflight move to center (Cartesian)...");
    if (move_group_) {
      const bool ok = moveToCenterCartesian();
      if (!ok && require_center_move_success_) {
        RCLCPP_ERROR(this->get_logger(), "Center move failed and require_center_move_success_=true. Not starting subscriptions.");
        return;
      }
      if (!ok) {
        RCLCPP_WARN(this->get_logger(), "Center move failed — continuing to start subscriptions anyway.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Move group not attached — skipping center move.");
    }

    // Start processing only after the (attempted) center move
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_.c_str(), 10, std::bind(&DepthDipDetector::pointcloudCallback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud: %s", input_topic_.c_str());
  }

private:
  // ---- Utility formatters -----------------------------------------------------
  static geometry_msgs::msg::Quaternion normalizeQuat(double x, double y, double z, double w)
  {
    geometry_msgs::msg::Quaternion q;
    const double n = std::sqrt(x*x + y*y + z*z + w*w);
    if (n < 1e-9) { q.x = q.y = q.z = 0.0; q.w = 1.0; }
    else { q.x = x/n; q.y = y/n; q.z = z/n; q.w = w/n; }
    return q;
  }

  static std::string poseToString(const geometry_msgs::msg::Pose &p)
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed); oss.precision(4);
    oss << "pos[" << p.position.x << ", " << p.position.y << ", " << p.position.z << "], "
        << "quat[" << p.orientation.x << ", " << p.orientation.y << ", "
        << p.orientation.z << ", " << p.orientation.w << "]";
    return oss.str();
  }

  void init_interfaces_()
  {
    pub_problematic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_.c_str(), 10);
    RCLCPP_INFO(this->get_logger(), "Advertising problematic cloud on: %s", output_topic_.c_str());
  }

  // ── EDIT THESE VARIABLES AS YOU LIKE ──────────────────────────────────────────
  // Topics
  std::string input_topic_  = "/wrist3_cam/points";
  std::string output_topic_ = "/problematic_points";

  // Parallelogram corners (x,y) in base frame
  const std::array<std::pair<double,double>,4> corners_ = {{
    {0.8, 0.4}, {1.2, 0.4}, {1.2, 0.0}, {0.8, 0.0}
  }};

  // Spraying & matching
  double spray_width_    = 0.04;   // grid spacing along X/Y for waypoints
  double match_radius_   = 0.02;   // waypoint proximity check (meters)

  // Robot base offset used earlier in your logic
  double robot_base_x_   = 0.25;
  double robot_base_y_   = 0.0;

  // “Problematic” depth thresholding
  double z_threshold_    = 0.07;
  double tolerance_      = 0.007;

  // Move-to-center settings
  std::string reference_frame_ = "base_link";
  double center_z_             = 0.60;

  // Your “look-down” quaternion (unit 180° about Y)
  double center_qx_ = 0.0, center_qy_ = 1.0, center_qz_ = 0.0, center_qw_ = 0.0;

  // Motion scaling
  double max_vel_scale_ = 0.30;
  double max_acc_scale_ = 0.30;

  // If true, abort subscribing if the move-to-center fails
  bool require_center_move_success_ = false;

  // Logging verbosity
  bool print_grid_once_ = false;        // will print grid stats once on first generation
  const size_t sample_print_limit_ = 10;// limit for sample lists
  // ─────────────────────────────────────────────────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_problematic_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // Stats
  size_t clouds_seen_ = 0;

  std::pair<double,double> parallelogramCenterXY() const
  {
    // Center of parallelogram = midpoint of diagonal (c0,c2)
    const double cx = 0.5 * (corners_[0].first  + corners_[2].first);
    const double cy = 0.5 * (corners_[0].second + corners_[2].second);
    return {cx, cy};
  }

  bool moveToCenterCartesian()
  {
    // Target pose (in base frame), compensate the robot_base_* offset like your waypoint logic
    auto [cx, cy] = parallelogramCenterXY();
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = cx - robot_base_x_;
    target_pose.position.y = cy - robot_base_y_;
    target_pose.position.z = center_z_;
    target_pose.orientation = normalizeQuat(center_qx_, center_qy_, center_qz_, center_qw_);

    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", reference_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target center (raw): (%.4f, %.4f), with base offset (-%.4f, -%.4f)",
                cx, cy, robot_base_x_, robot_base_y_);
    RCLCPP_INFO(this->get_logger(), "Target pose: %s", poseToString(target_pose).c_str());

    // Cartesian path from current pose to target_pose
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setMaxVelocityScalingFactor(max_vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(max_acc_scale_);
    move_group_->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);  // MoveIt will interpolate with eef_step

    moveit_msgs::msg::RobotTrajectory traj_msg;
    const double eef_step = 0.01;       // meters resolution
    const double jump_threshold = 0.0;  // disable jump threshold
    const bool avoid_collisions = true;

    double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, traj_msg, avoid_collisions);

    RCLCPP_INFO(this->get_logger(), "computeCartesianPath: fraction=%.3f, points=%zu",
                fraction, traj_msg.joint_trajectory.points.size());

    if (fraction < 0.99) {
      RCLCPP_WARN(this->get_logger(),
                  "Cartesian path to center incomplete: fraction=%.3f (want ~1.0).", fraction);
      if (fraction <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute any Cartesian path.");
        return false;
      }
    }

    // Time-parameterize the trajectory for smooth execution
    {
      auto robot_model = move_group_->getRobotModel();
      auto current_state = move_group_->getCurrentState(5.0);
      if (!current_state) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state for time parameterization.");
        return false;
      }
      robot_trajectory::RobotTrajectory rt(robot_model, move_group_->getName());
      rt.setRobotTrajectoryMsg(*current_state, traj_msg);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      const bool timed = iptp.computeTimeStamps(rt, max_vel_scale_, max_acc_scale_);
      if (!timed) {
        RCLCPP_WARN(this->get_logger(), "Time parameterization failed; executing un-timed trajectory.");
      } else {
        rt.getRobotTrajectoryMsg(traj_msg);
        const auto &pts = traj_msg.joint_trajectory.points;
        double total_time = pts.empty() ? 0.0 : pts.back().time_from_start.sec +
                                                pts.back().time_from_start.nanosec * 1e-9;
        RCLCPP_INFO(this->get_logger(), "Time-param OK: points=%zu, total_time=%.3fs",
                    pts.size(), total_time);
      }
    }

    // Execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj_msg;
    auto exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "Execute Cartesian path: %s", exec_ok ? "SUCCESS" : "FAIL");
    if (!exec_ok) {
      RCLCPP_ERROR(this->get_logger(), "Execution of Cartesian path to center failed.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
      "Reached center via Cartesian path: %s", poseToString(target_pose).c_str());
    return true;
  }

  std::vector<std::pair<double,double>> generateWaypoints()
  {
    const double dx = corners_[1].first  - corners_[0].first;
    const double dy = corners_[3].second - corners_[0].second;

    const double length = std::hypot(dx, corners_[1].second - corners_[0].second);
    const double height = std::hypot(corners_[3].first - corners_[0].first, dy);

    int cols = std::max(1, static_cast<int>(std::floor(length / spray_width_)));
    int rows = std::max(1, static_cast<int>(std::floor(height / spray_width_)));

    const double step_x = dx / static_cast<double>(cols);
    const double step_y = dy / static_cast<double>(rows);

    if (!print_grid_once_) {
      RCLCPP_INFO(this->get_logger(),
                  "Grid stats: length=%.4f, height=%.4f, cols=%d, rows=%d, step_x=%.4f, step_y=%.4f",
                  length, height, cols, rows, step_x, step_y);
      print_grid_once_ = true;
    }

    std::vector<std::pair<double,double>> positions;
    positions.reserve(static_cast<size_t>(rows) * static_cast<size_t>(cols));

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        int col = (i % 2 == 0) ? j : (cols - j - 1);
        double x = corners_[0].first + (static_cast<double>(col) + 0.5) * step_x - robot_base_x_;
        double y = corners_[0].second + (static_cast<double>(i) + 0.5) * step_y - robot_base_y_;
        positions.emplace_back(std::round(x * 1e4) / 1e4, std::round(y * 1e4) / 1e4);
      }
    }

    // Print a few sample waypoints
    if (!positions.empty()) {
      size_t n_show = std::min(sample_print_limit_, positions.size());
      std::ostringstream oss;
      oss.setf(std::ios::fixed); oss.precision(4);
      oss << "Waypoint samples (" << n_show << "/" << positions.size() << "): ";
      for (size_t i = 0; i < n_show; ++i) {
        oss << "(" << positions[i].first + robot_base_x_ << ", "  // rebase for readability
               << positions[i].second + robot_base_y_ << ")";
        if (i + 1 < n_show) oss << ", ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    return positions;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++clouds_seen_;

    // Summary of the input cloud
    const auto &h = msg->header;
    const size_t npts = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    RCLCPP_INFO(this->get_logger(),
      "[Cloud %zu] frame='%s' stamp=%u.%09u size=%ux%u (%zu pts) | z_thresh=%.4f tol=%.4f",
      clouds_seen_, h.frame_id.c_str(), h.stamp.sec, h.stamp.nanosec,
      msg->width, msg->height, npts, z_threshold_, tolerance_);

    // Iterate and collect problematic points, while tracking min/max/avg Z
    std::vector<std::array<float,3>> problematic_points;
    problematic_points.reserve(npts);

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double sum_z = 0.0;
    size_t valid_count = 0;

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      // stats
      min_z = std::min(min_z, static_cast<double>(z));
      max_z = std::max(max_z, static_cast<double>(z));
      sum_z += z;
      ++valid_count;

      if (static_cast<double>(z) < (z_threshold_ - tolerance_)) {
        problematic_points.push_back({x, y, z});
      }
    }

    const double avg_z = valid_count ? (sum_z / static_cast<double>(valid_count)) : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(this->get_logger(),
      "[Cloud %zu] valid=%zu minZ=%.4f maxZ=%.4f avgZ=%.4f | problematic=%zu",
      clouds_seen_, valid_count, min_z, max_z, avg_z, problematic_points.size());

    // Print up to N sample problematic points
    if (!problematic_points.empty()) {
      size_t n_show = std::min(sample_print_limit_, problematic_points.size());
      std::ostringstream oss;
      oss.setf(std::ios::fixed); oss.precision(4);
      oss << "Problematic sample (" << n_show << "/" << problematic_points.size() << "): ";
      for (size_t i = 0; i < n_show; ++i) {
        const auto &p = problematic_points[i];
        oss << "(" << p[0] << ", " << p[1] << ", " << p[2] << ")";
        if (i + 1 < n_show) oss << ", ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    // Waypoint matching
    const auto waypoints = generateWaypoints();
    std::set<std::pair<double,double>> matched_waypoints;

    for (const auto &p : problematic_points) {
      const double px = p[0], py = p[1];
      for (const auto &wp : waypoints) {
        if (std::hypot(px - wp.first, py - wp.second) <= match_radius_) {
          matched_waypoints.insert(wp);
          break;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "[Cloud %zu] matched_waypoints=%zu (radius=%.3f)",
                clouds_seen_, matched_waypoints.size(), match_radius_);

    if (!matched_waypoints.empty()) {
      size_t n_show = std::min(sample_print_limit_, matched_waypoints.size());
      std::ostringstream oss;
      oss.setf(std::ios::fixed); oss.precision(4);
      oss << "Matched waypoint samples (" << n_show << "/" << matched_waypoints.size() << "): ";
      size_t i = 0;
      for (const auto &wp : matched_waypoints) {
        // Rebase for readability in base frame coordinates
        const double x = wp.first  + robot_base_x_;
        const double y = wp.second + robot_base_y_;
        oss << "(" << x << ", " << y << ")";
        if (++i >= n_show) break;
        oss << ", ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    // Publish problematic points in base_link
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;                 // keep stamp
    out.header.frame_id = "base_link";        // as in your original code

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    mod.resize(problematic_points.size());

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (const auto &p : problematic_points) {
      *ox = p[0]; *oy = p[1]; *oz = p[2];
      ++ox; ++oy; ++oz;  // prefix increments (postfix++ is not defined for these iterators)
    }

    pub_problematic_->publish(out);
    RCLCPP_INFO(this->get_logger(),
      "[Cloud %zu] published %zu problematic points to %s (frame=base_link)",
      clouds_seen_, problematic_points.size(), output_topic_.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true) });
  auto node = std::make_shared<DepthDipDetector>(options);

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "ur_manipulator");
  node->attach_move_group(move_group);

  // Spin executor on a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([executor]() {
    RCLCPP_INFO(rclcpp::get_logger("executor"), "Executor spinning...");
    executor->spin();  // returns when rclcpp::shutdown() happens (e.g., Ctrl+C)
    RCLCPP_INFO(rclcpp::get_logger("executor"), "Executor stopped.");
  });

  node->move_to_center_and_start();

  // Optional: ensure the executor stops when shutdown is triggered
  rclcpp::on_shutdown([executor]() { executor->cancel(); });

  // BLOCK here until shutdown (Ctrl+C). No rclcpp::wait_for_shutdown().
  executor_thread.join();

  // (Usually already shut down by the SIGINT handler)
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
