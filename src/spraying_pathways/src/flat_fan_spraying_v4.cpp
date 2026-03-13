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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <moveit/robot_state/conversions.h>

using moveit::core::RobotState;

using namespace std;
using namespace Eigen;

struct Point2D {
  double x, y;
  Point2D() : x(0), y(0) {}
  Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
  bool operator<(const Point2D& other) const {
    if (y != other.y) return y > other.y;
    return x < other.x;
  }
};

struct Pose3D {
  Point2D position;
  double z;
};

struct Cube {
  int id;
  Pose3D pose;
  double height;
};
///////////////////////////////////////////////////////////////////////////////////////////
// --- Utility Functions ---
Vector2d toVector(const Point2D& p) {
    return Vector2d(p.x, p.y);
}

Point2D toPoint2D(const Vector2d& v) {
    return {v.x(), v.y()};
}

Vector2d unit_vector(const Point2D& p0, const Point2D& p1) {
    Vector2d vec = toVector(p1) - toVector(p0);
    double norm = vec.norm();
    if (norm == 0.0) throw runtime_error("Zero-length vector.");
    return vec / norm;
}

double compute_accel_distance(double v_target, double a_max) {
    return (v_target * v_target) / (2.0 * a_max);
}

Point2D offset_point(const Point2D& p, const Vector2d& dir, double dist) {
    return toPoint2D(toVector(p) - dir * dist);
}

tuple<Point2D, Point2D, double> compute_start_end(const vector<Point2D>& spray_pts, double v_target, double a_max) {
    double d = compute_accel_distance(v_target, a_max);
    Vector2d spray_dir = unit_vector(spray_pts.front(), spray_pts.back());
    Point2D start = offset_point(spray_pts.front(), spray_dir, d);
    Point2D end = offset_point(spray_pts.back(), -spray_dir, d);
    return {start, end, d};
}

tuple<vector<double>, vector<double>, vector<double>, double> smooth_velocity_profile_exact_a(double v_target, double a_max, int steps = 100) {
    double T = 1.5 * v_target / a_max;
    vector<double> t(steps), v(steps), a(steps);
    for (int i = 0; i < steps; ++i) {
        t[i] = (T * i) / (steps - 1);
        double tau = t[i] / T;
        v[i] = v_target * (3 * tau * tau - 2 * tau * tau * tau);
        a[i] = v_target * (6 * tau - 6 * tau * tau) / T;
    }
    return {t, v, a, T};
}

tuple<vector<double>, vector<Point2D>, vector<double>, vector<double>, Point2D, vector<Point2D>, Point2D>
generate_trajectory(const vector<Point2D>& spray_pts, double v_target, double a_max, int steps = 100) {
    auto [start, end, d] = compute_start_end(spray_pts, v_target, a_max);
    Vector2d dir_vec = unit_vector(spray_pts.front(), spray_pts.back());
    double L = (toVector(spray_pts.back()) - toVector(spray_pts.front())).norm();

    // Acceleration phase
    auto [t1, v1, a1, T1] = smooth_velocity_profile_exact_a(v_target, a_max, steps);
    vector<double> x1(steps);
    for (int i = 0; i < steps; ++i)
        x1[i] = (i == 0 ? 0 : x1[i - 1]) + v1[i] * (T1 / steps);
    double scale1 = d / x1.back();
    for (auto& x : x1) x *= scale1;

    // Constant velocity phase
    double T2 = L / v_target;
    vector<double> t2(steps), x2(steps), v2(steps, v_target), a2(steps, 0);
    for (int i = 0; i < steps; ++i) {
        t2[i] = T2 * i / (steps - 1);
        x2[i] = x1.back() + (L * i) / (steps - 1);
    }

    // Deceleration phase
    auto [t3, v3, a3, T3] = smooth_velocity_profile_exact_a(v_target, a_max, steps);
    vector<double> x3(steps);
    for (int i = 0; i < steps; ++i)
        x3[i] = (i == 0 ? 0 : x3[i - 1]) + v3[steps - 1 - i] * (T3 / steps);
    double scale3 = d / x3.back();
    for (auto& x : x3) x = x2.back() + x * scale3;

    // Concatenate time
    vector<double> t_all;
    t_all.insert(t_all.end(), t1.begin(), t1.end());
    for (auto& t_val : t2) t_all.push_back(t1.back() + t_val);
    for (auto& t_val : t3) t_all.push_back(t1.back() + T2 + t_val);

    // Concatenate position
    vector<double> x_all;
    x_all.insert(x_all.end(), x1.begin(), x1.end());
    x_all.insert(x_all.end(), x2.begin(), x2.end());
    x_all.insert(x_all.end(), x3.begin(), x3.end());

    // Concatenate velocity
    vector<double> v_all;
    v_all.insert(v_all.end(), v1.begin(), v1.end());
    v_all.insert(v_all.end(), v2.begin(), v2.end());
    for (int i = steps - 1; i >= 0; --i) v_all.push_back(v3[i]);

    // Concatenate acceleration
    vector<double> a_all;
    a_all.insert(a_all.end(), a1.begin(), a1.end());
    a_all.insert(a_all.end(), a2.begin(), a2.end());
    for (int i = steps - 1; i >= 0; --i) a_all.push_back(-a3[i]);

    // Build 2D trajectory
    vector<Point2D> xy;
    Vector2d origin = toVector(start);
    for (const auto& x : x_all)
        xy.emplace_back(toPoint2D(origin + dir_vec * x));

    return {t_all, xy, v_all, a_all, start, spray_pts, end};
}


tuple<vector<double>, vector<Point2D>, vector<double>, vector<double>> smooth_cartesian_transition(const Point2D& p0, const Point2D& p1, double a_max, int steps = 100, double tol = 1e-4) {
    double d = (toVector(p1) - toVector(p0)).norm();
    Vector2d direction = unit_vector(p0, p1);

    auto quintic_profile = [](double t, double T) -> tuple<double, double, double> {
        double tau = t / T;
        double pos = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5);
        double vel = (30 * pow(tau, 2) - 60 * pow(tau, 3) + 30 * pow(tau, 4)) / T;
        double acc = (60 * tau - 180 * pow(tau, 2) + 120 * pow(tau, 3)) / (T * T);
        return {pos, vel, acc};
    };

    double T_min = sqrt(15 * d / (8 * a_max)) * 0.5;
    double T_max = sqrt(15 * d / (8 * a_max)) * 2.0;

    while (T_max - T_min > tol) {
        double T = (T_min + T_max) / 2.0;
        bool valid = true;
        for (int i = 0; i < steps; ++i) {
            double t = T * i / (steps - 1);
            auto [_, __, a] = quintic_profile(t, T);
            if (abs(a * d) > a_max) {
                valid = false;
                break;
            }
        }
        if (valid)
            T_max = (T_min + T_max) / 2.0;
        else
            T_min = (T_min + T_max) / 2.0;
    }

    double T = T_max;
    vector<double> t(steps), v(steps), a(steps);
    vector<Point2D> xy;
    for (int i = 0; i < steps; ++i) {
        t[i] = T * i / (steps - 1);
        auto [pos, vel, acc] = quintic_profile(t[i], T);
        double x = pos * d;
        xy.emplace_back(toPoint2D(toVector(p0) + direction * x));
        v[i] = vel * d;
        a[i] = acc * d;
    }
    return {t, xy, v, a};
}

tuple<vector<double>, vector<Point2D>, vector<double>, vector<double>> generate_multi_line_trajectory(const vector<vector<Point2D>>& lines, double v_target, double a_max, int steps = 100) {
    vector<double> t_all, v_all, a_all;
    vector<Point2D> xy_all;
    double t_offset = 0.0;

    for (size_t i = 0; i < lines.size(); ++i) {
        auto [t, xy, v, a, start, spray_pts, end] = generate_trajectory(lines[i], v_target, a_max, steps);
        for (auto& val : t) t_all.push_back(val + t_offset);
        xy_all.insert(xy_all.end(), xy.begin(), xy.end());
        v_all.insert(v_all.end(), v.begin(), v.end());
        a_all.insert(a_all.end(), a.begin(), a.end());
        t_offset = t_all.back();

        if (i + 1 < lines.size()) {
            auto [next_start, __, ___] = compute_start_end(lines[i + 1], v_target, a_max);
            auto [t_tr, xy_tr, v_tr, a_tr] = smooth_cartesian_transition(end, next_start, a_max, steps);
            for (auto& val : t_tr) t_all.push_back(val + t_offset);
            xy_all.insert(xy_all.end(), xy_tr.begin(), xy_tr.end());
            v_all.insert(v_all.end(), v_tr.begin(), v_tr.end());
            a_all.insert(a_all.end(), a_tr.begin(), a_tr.end());
            t_offset = t_all.back();
        }
    }
    return {t_all, xy_all, v_all, a_all};
}

// --- Nearest Point Time Query ---
double get_time_from_position_nearest(const vector<Point2D>& xy_traj, const vector<double>& t_traj, const Point2D& query_xy) {
    double min_dist = numeric_limits<double>::max();
    size_t nearest_idx = 0;

    for (size_t i = 0; i < xy_traj.size(); ++i) {
        double dx = xy_traj[i].x - query_xy.x;
        double dy = xy_traj[i].y - query_xy.y;
        double dist = dx * dx + dy * dy;

        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return t_traj[nearest_idx];
}

std::vector<std::vector<Point2D>> organize_waypoints_into_spray_lines(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double y_tolerance = 1e-4
) {
    std::vector<std::vector<Point2D>> lines;

    // Group poses into lines based on similar Y values
    for (const auto& pose : waypoints) {
        Point2D pt{pose.position.x, pose.position.y};
        bool added = false;
        for (auto& line : lines) {
            if (!line.empty() && std::abs(line.front().y - pt.y) < y_tolerance) {
                line.push_back(pt);
                added = true;
                break;
            }
        }
        if (!added) {
            lines.push_back({pt});
        }
    }

    // Sort lines by Y descending (top to bottom)
    std::sort(lines.begin(), lines.end(), [](const auto& a, const auto& b) {
        return a.front().y > b.front().y;
    });

    // Zig-zag sort within each line by X (even lines: increasing, odd lines: decreasing)
    for (size_t i = 0; i < lines.size(); ++i) {
        auto& line = lines[i];
        std::sort(line.begin(), line.end(), [](const Point2D& a, const Point2D& b) {
            return a.x < b.x;
        });
        if (i % 2 == 1) {
            std::reverse(line.begin(), line.end());
        }
    }

    return lines;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& points) {
  if (points.size() != 4) throw std::runtime_error("Exactly 4 points required");
  auto sorted = points;
  std::sort(sorted.begin(), sorted.end());
  std::vector<Point2D> top(sorted.begin(), sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin() + 2, sorted.end());
  std::sort(top.begin(), top.end(), [](const Point2D& a, const Point2D& b) { return a.x < b.x; });
  std::sort(bottom.begin(), bottom.end(), [](const Point2D& a, const Point2D& b) { return a.x < b.x; });
  return {top[0], top[1], bottom[1], bottom[0]};
}

void generate_grid(const std::vector<Point2D>& area_corners,
                   double spray_width,
                   double spray_length,
                   int N,
                   std::vector<Cube>& cubes,
                   std::vector<Point2D>& spray_centers,
                   double& cube_size_x,
                   double& cube_size_y) {
  double dx = area_corners[1].x - area_corners[0].x;
  double dy = area_corners[3].y - area_corners[0].y;

  double total_width = std::hypot(dx, area_corners[1].y - area_corners[0].y);
  double total_length = std::hypot(area_corners[3].x - area_corners[0].x, dy);

  int cols = std::max(1, static_cast<int>(total_width / spray_width));
  int rows = std::max(1, static_cast<int>(total_length / spray_length));

  double step_x = dx / cols;
  double step_y = dy / rows;

  cube_size_x = step_x;
  cube_size_y = step_y / N;

  cubes.clear();
  spray_centers.clear();
  int cube_id = 0;

  for (int i = 0; i < rows; ++i) {
    std::vector<Point2D> row_centers;
    for (int j = 0; j < cols; ++j) {
      double patch_origin_x = area_corners[0].x + j * step_x;
      double patch_origin_y = area_corners[0].y + i * step_y;

      for (int n = 0; n < N; ++n) {
        double center_x = patch_origin_x + step_x / 2.0;
        double center_y = patch_origin_y + (n + 0.5) * cube_size_y;
        cubes.push_back({cube_id++, {{center_x, center_y}, 0.0}, 0.0});
      }

      int mid_idx = N / 2;
      double waypoint_y = patch_origin_y + (mid_idx + 0.5) * cube_size_y;
      row_centers.push_back({patch_origin_x + step_x / 2.0, waypoint_y});
    }
    if (i % 2 == 1) std::reverse(row_centers.begin(), row_centers.end());
    spray_centers.insert(spray_centers.end(), row_centers.begin(), row_centers.end());
  }
}

void apply_flat_spray(std::vector<Cube>& cubes,
                      const std::vector<Point2D>& spray_centers,
                      double cube_size_x, double cube_size_y,
                      double radius, double standard_h, double min_h, double sigma, double z_base) {
  double epsilon = cube_size_x / 100.0;

  for (const auto& sc : spray_centers) {
    for (auto& cube : cubes) {
      double dx = std::abs(cube.pose.position.x - sc.x);
      double dy = std::abs(cube.pose.position.y - sc.y);

      if (dy <= radius && dx <= epsilon) {
        double h = (sigma == 0)
                   ? standard_h
                   : standard_h - (standard_h - min_h) * std::pow(dy / radius, sigma);
        cube.height += h;
        cube.pose.z = z_base + h / 2.0;
      }
    }
  }
}

std::string generate_multi_box_sdf(const std::vector<Cube>& cubes, double cube_size_x, double cube_size_y) {
  std::ostringstream sdf;
  sdf << "<?xml version='1.0'?>\n<sdf version='1.7'>\n<model name='multi_cube'>\n  <static>true</static>\n";

  for (const auto& cube : cubes) {
    sdf << "  <link name='cube_" << cube.id << "'>\n";
    sdf << "    <pose>"
        << cube.pose.position.x << " "
        << cube.pose.position.y << " "
        << cube.pose.z << " 0 0 0</pose>\n";
    sdf << "    <collision name='collision'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>"
        << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n    </collision>\n";
    sdf << "    <visual name='visual'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>"
        << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n";
    sdf << "      <material>\n        <script>\n";
    sdf << "          <uri>file:///ros2_ws/src/spraying_pathways/materials/scripts</uri>\n";
    sdf << "          <name>My/Seaweed</name>\n";
    sdf << "        </script>\n";
    sdf << "        <ambient>1 1 1 0.7</ambient>\n";
    sdf << "        <diffuse>1 1 1 0.7</diffuse>\n";
    sdf << "      </material>\n    </visual>\n  </link>\n";
  }

  sdf << "</model>\n</sdf>\n";
  return sdf.str();
}

using FollowJT = control_msgs::action::FollowJointTrajectory;

// === Gate που κάνει cancel on hazard ===
class UnknownGate
{
public:
  explicit UnknownGate(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    unknown_topic_   = node_->declare_parameter<std::string>("unknown_topic", "/unknown_points");
    min_points_      = node_->declare_parameter<int>("min_points", 1);
    debounce_ms_     = node_->declare_parameter<int>("debounce_ms", 200);
    controller_ns_   = node_->declare_parameter<std::string>("controller_ns", "joint_trajectory_controller");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      unknown_topic_, qos,
      std::bind(&UnknownGate::cbCloud, this, std::placeholders::_1));

    const std::string action_name = "/" + controller_ns_ + "/follow_joint_trajectory";
    traj_client_ = rclcpp_action::create_client<FollowJT>(node_, action_name);

    hold_client_ = node_->create_client<std_srvs::srv::Trigger>("/" + controller_ns_ + "/hold_position");

    RCLCPP_INFO(node_->get_logger(), "[Gate] watching '%s' (min=%d), controller_ns='%s'",
                unknown_topic_.c_str(), min_points_, controller_ns_.c_str());
  }

  void hardStop()
  {
    const auto now = node_->get_clock()->now();
    if ((now - last_stop_time_) < rclcpp::Duration::from_seconds(debounce_ms_/1000.0)) return;

    if (traj_client_ && traj_client_->wait_for_action_server(std::chrono::milliseconds(50))) {
      (void)traj_client_->async_cancel_all_goals();
      we_cancelled_.store(true, std::memory_order_relaxed);
      RCLCPP_WARN(node_->get_logger(), "[Gate] CANCEL ALL GOALS");
    }
    if (hold_client_ && hold_client_->wait_for_service(std::chrono::milliseconds(50))) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)hold_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "[Gate] hold_position requested");
    }
    last_stop_time_ = now;
  }

  bool hazard() const { return has_unknown_.load(std::memory_order_relaxed); }
  void clearCancelFlag() { we_cancelled_.store(false, std::memory_order_relaxed); }
  bool wasCancelled() const { return we_cancelled_.load(std::memory_order_relaxed); }

private:
  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const bool danger = (n >= static_cast<size_t>(min_points_));
    has_unknown_.store(danger, std::memory_order_relaxed);
    if (danger) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[Gate] Unknown: %zu pts -> HAZARD", n);
      hardStop();
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp_action::Client<FollowJT>::SharedPtr traj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_client_;
  std::string unknown_topic_, controller_ns_;
  int min_points_{1}, debounce_ms_{200};
  std::atomic<bool> has_unknown_{false};
  std::atomic<bool> we_cancelled_{false};
  rclcpp::Time last_stop_time_{0,0,RCL_ROS_TIME};
};

// Βρες κοντινότερο index σε joint-space
static size_t nearest_joint_index(
  const trajectory_msgs::msg::JointTrajectory& jt,
  const std::vector<double>& q,
  size_t start_idx = 0)
{
  auto dist2 = [&](const std::vector<double>& a, const std::vector<double>& b){
    double s=0; for (size_t i=0;i<a.size();++i){ double d=a[i]-b[i]; s+=d*d; } return s;
  };
  size_t best = start_idx;
  double bestd = std::numeric_limits<double>::infinity();
  for (size_t i=start_idx; i<jt.points.size(); ++i) {
    if (jt.points[i].positions.size()!=q.size()) continue;
    double d = dist2(jt.points[i].positions, q);
    if (d < bestd){ bestd=d; best=i; }
  }
  return best;
}

// Ράψε remainder ώστε να ξεκινά από ΤΩΡΑ (t=0) + δώσε αρκετό χρόνο στο 1ο στόχο
static trajectory_msgs::msg::JointTrajectory slice_and_retime(
  const trajectory_msgs::msg::JointTrajectory& src,
  size_t from_idx,
  moveit::planning_interface::MoveGroupInterface& mgi,
  const std::string& planning_group,
  double min_dt = 0.3,
  double vel_fraction = 0.5,
  double default_vmax = 1.0 // rad/s (fallback)
)
{
  trajectory_msgs::msg::JointTrajectory out;
  out.joint_names = src.joint_names;
  if (from_idx >= src.points.size()) return out;

  // Τρέχουσα στάση
  moveit::core::RobotStatePtr st = mgi.getCurrentState(0.5);
  const moveit::core::JointModelGroup* jmg = st->getJointModelGroup(planning_group);
  std::vector<double> q_now; st->copyJointGroupPositions(jmg, q_now);

  // Εκτίμηση χρόνου για τον 1ο στόχο του remainder
  const auto& first_goal = src.points[from_idx];
  double dt = min_dt;
  if (first_goal.positions.size() == q_now.size()) {
    double max_dq = 0.0;
    for (size_t i=0; i<q_now.size(); ++i) {
      max_dq = std::max(max_dq, std::abs(first_goal.positions[i] - q_now[i]));
    }
    double need = max_dq / (vel_fraction * default_vmax);
    if (need > 0.0) dt = std::max(dt, need);
  }

  // Χρόνος αναφοράς του from_idx στην original
  rclcpp::Duration t0(src.points[from_idx].time_from_start.sec,
                      src.points[from_idx].time_from_start.nanosec);

  // p0 = τρέχουσα στάση, t=0
  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.positions = q_now;
  p0.time_from_start.sec = 0;
  p0.time_from_start.nanosec = 0;
  out.points.push_back(std::move(p0));

  // Τα υπόλοιπα σημεία με offset +dt
  for (size_t i = from_idx; i < src.points.size(); ++i) {
    auto p = src.points[i];
    rclcpp::Duration ti(p.time_from_start.sec, p.time_from_start.nanosec);
    rclcpp::Duration rel = ti - t0 + rclcpp::Duration::from_seconds(dt);

    // εξασφάλισε >0 στο πρώτο πραγματικό σημείο
    if (i == from_idx && (rel.nanoseconds() <= 0)) {
      rel = rclcpp::Duration::from_seconds(dt);
    }

    int64_t ns = rel.nanoseconds();
    int32_t sec = static_cast<int32_t>(ns / 1000000000LL);
    uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);
    p.time_from_start.sec = sec;
    p.time_from_start.nanosec = nsec;

    p.velocities.clear();
    p.accelerations.clear();
    out.points.push_back(std::move(p));
  }

  return out;
}

// Εκτέλεση με cancel/hold και αυτόματο resume του "υπολοίπου"
static void execute_with_pause_resume(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const moveit_msgs::msg::RobotTrajectory& full_msg,
  UnknownGate& gate,
  const std::string& planning_group)
{
  trajectory_msgs::msg::JointTrajectory full = full_msg.joint_trajectory;
  size_t start_idx = 0;

  while (start_idx < full.points.size()) {
    // Περίμενε να καθαρίσει
    while (gate.hazard()) std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Φτιάξε remainder που ξεκινά από ΤΩΡΑ
    trajectory_msgs::msg::JointTrajectory remainder =
        slice_and_retime(full, start_idx, mgi, planning_group, 0.3, 0.5);
    if (remainder.points.size() < 2) break;

    moveit_msgs::msg::RobotTrajectory msg2;
    msg2.joint_trajectory = remainder;

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    plan2.trajectory_ = msg2;

    // start_state = τρέχουσα στάση (βοηθά στα tolerances)
    {
      moveit::core::RobotStatePtr st = mgi.getCurrentState(0.2);
      moveit::core::robotStateToRobotStateMsg(*st, plan2.start_state_);
    }

    gate.clearCancelFlag();
    RCLCPP_INFO(rclcpp::get_logger("spray_sim_node"),
                "Executing chunk from idx=%zu (%zu pts remaining)...",
                start_idx, remainder.points.size()-1);

    // μικρή καθυστέρηση μετά από cancel
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    auto ret = mgi.execute(plan2);

    if (gate.wasCancelled() || gate.hazard() ||
        ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
      while (gate.hazard()) std::this_thread::sleep_for(std::chrono::milliseconds(20));

      moveit::core::RobotStatePtr st = mgi.getCurrentState(1.0);
      const moveit::core::JointModelGroup* jmg = st->getJointModelGroup(planning_group);
      std::vector<double> q; st->copyJointGroupPositions(jmg, q);
      size_t near_idx = nearest_joint_index(full, q, start_idx);
      start_idx = std::min(near_idx + 1, full.points.size());
      RCLCPP_WARN(rclcpp::get_logger("spray_sim_node"),
                  "Paused/cancelled -> resume from nearest idx=%zu", start_idx);
      continue;
    } else {
      // τελείωσε αυτό το remainder χωρίς διακοπή
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("spray_sim_node"),
              "Execution finished (pause/resume capable).");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

// --- Main ---
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

    std::vector<Point2D> unordered = {{0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4}};
    std::vector<Point2D> corners = sort_rectangle_corners(unordered);

    const double robot_base_x = 0.25;
    const double robot_base_y = 0.0;

    double z_base = 0.765;
    double spray_width = 0.02;
    double spray_length = 0.1;
    int N = 9;
    double radius = spray_length / 2;
    double standard_h = 0.015;
    double min_h = 0.009;
    double sigma = 3;

    double cube_size_x, cube_size_y;
    std::vector<Cube> cubes;
    std::vector<Point2D> spray_centers;

    generate_grid(corners, spray_width, spray_length, N, cubes, spray_centers, cube_size_x, cube_size_y);
    cube_size_x = std::abs(cube_size_x);
    cube_size_y = std::abs(cube_size_y);

    apply_flat_spray(cubes, spray_centers, cube_size_x, cube_size_y, radius, standard_h, min_h, sigma, z_base);

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = 0.0;
    orientation.y = 1.0;
    orientation.z = 0.0;
    orientation.w = 0.0;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& sc : spray_centers) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = sc.x - robot_base_x;
        pose.position.y = sc.y - robot_base_y;
        pose.position.z = 0.2;
        pose.orientation = orientation;
        waypoints.push_back(pose);
    }

    auto spray_lines = organize_waypoints_into_spray_lines(waypoints);
    for (size_t i = 0; i < spray_lines.size(); ++i) {
        const auto& line = spray_lines[i];
        RCLCPP_INFO(rclcpp::get_logger("spray_logger"), "Spray line %zu: %zu points", i, line.size());
        for (size_t j = 0; j < line.size(); ++j) {
            RCLCPP_INFO(rclcpp::get_logger("spray_logger"), "  Point %zu: (%.3f, %.3f)", j, line[j].x, line[j].y);
        }
    }

    auto [t, xy, v, a] = generate_multi_line_trajectory(spray_lines, 0.05, 0.03, 100);

    // Save trajectory data to CSV in current working directory
    std::ofstream file("trajectory_output.csv");
    if (file.is_open()) {
        file << "time [s],x [m],y [m],speed [m/s],acceleration [m/s²]\n";
        for (size_t i = 0; i < t.size(); ++i) {
            file << t[i] << "," << xy[i].x << "," << xy[i].y << "," << v[i] << "," << a[i] << "\n";
            RCLCPP_INFO(node->get_logger(), "[Time %.3f s]  x[%f] y[%f] v[%f] a[%f]", t[i], xy[i].x, xy[i].y, v[i], a[i]);
        }
        file.close();
        RCLCPP_INFO(node->get_logger(), "Trajectory saved to 'trajectory_output.csv'");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to write to 'trajectory_output.csv'");
    }

    RCLCPP_INFO(node->get_logger(), "[First point  x[%f] y[%f]", xy[0].x, xy[0].y);

    // === Move to first trajectory point ===
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setPlanningTime(60.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = xy[0].x;
    target_pose.position.y =  xy[0].y;
    target_pose.position.z = 0.2;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    std::vector<geometry_msgs::msg::Pose> single_waypoint;
    single_waypoint.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group.computeCartesianPath(single_waypoint, eef_step, jump_threshold, trajectory);

    if (fraction > 0.95) {
        RCLCPP_INFO(node->get_logger(), "Cartesian path planned successfully (%.1f%% achieved)", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
    } else {
        RCLCPP_WARN(node->get_logger(), "Only %.1f%% of Cartesian path was planned", fraction * 100.0);
    }

    std::vector<geometry_msgs::msg::Pose> trajectory_waypoints;
    for (size_t i = 0; i < xy.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = xy[i].x;
        pose.position.y = xy[i].y;
        pose.position.z = 0.2;  // fixed height
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        trajectory_waypoints.push_back(pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory_2;

    double fraction_2 = move_group.computeCartesianPath(trajectory_waypoints, eef_step, jump_threshold, trajectory_2);
    RCLCPP_INFO(node->get_logger(), "Cartesian path planning completed %.2f%% of the path", fraction_2 * 100.0);

    const auto& joint_names = trajectory_2.joint_trajectory.joint_names;
    const auto& joint_points = trajectory_2.joint_trajectory.points;

    RCLCPP_INFO(node->get_logger(), "Extracted %zu joint points from trajectory.", joint_points.size());

    for (size_t i = 0; i < joint_points.size(); ++i) {
        const auto& point = joint_points[i];
        RCLCPP_INFO(node->get_logger(), "Waypoint %zu:", i);
        for (size_t j = 0; j < point.positions.size(); ++j) {
            RCLCPP_INFO(node->get_logger(), "  %s: %.6f", joint_names[j].c_str(), point.positions[j]);
        }
    }

    // === FK Logging for each joint point ===
    const std::string planning_group = move_group.getName();
    const std::string end_effector_link = move_group.getEndEffectorLink();

    moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group =
        kinematic_state->getJointModelGroup(planning_group);

    for (size_t i = 0; i < joint_points.size(); ++i) {
        kinematic_state->setJointGroupPositions(joint_model_group, joint_points[i].positions);
        const Eigen::Isometry3d& ee_transform = kinematic_state->getGlobalLinkTransform(end_effector_link);
        geometry_msgs::msg::Pose ee_pose = tf2::toMsg(ee_transform);

        RCLCPP_INFO(node->get_logger(), "[Waypoint %zu] FK Pose:", i);
        RCLCPP_INFO(node->get_logger(), "  Position -> x: %.6f, y: %.6f, z: %.6f",
                    ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "  Orientation -> x: %.6f, y: %.6f, z: %.6f, w: %.6f",
                    ee_pose.orientation.x, ee_pose.orientation.y,
                    ee_pose.orientation.z, ee_pose.orientation.w);
        RCLCPP_INFO(node->get_logger(), "--------------------------------------------");
    }
    //////////////////////////////////////////////////////////////
    robot_trajectory::RobotTrajectory robot_traj(move_group.getRobotModel(), planning_group);

    double last_time = 0.0;

    // Build the trajectory using FK-based Cartesian query + timing
    for (size_t i = 0; i < joint_points.size(); ++i) {
        kinematic_state->setJointGroupPositions(joint_model_group, joint_points[i].positions);
        const Eigen::Isometry3d& tf = kinematic_state->getGlobalLinkTransform(end_effector_link);
        Point2D query_xy{tf.translation().x(), tf.translation().y()};

        double matched_time = get_time_from_position_nearest(xy, t, query_xy);
        double delta_time = matched_time - last_time;
        last_time = matched_time;

        robot_traj.addSuffixWayPoint(*kinematic_state, delta_time);

        RCLCPP_INFO(node->get_logger(),
            "[Waypoint %zu] FK xy(%.4f, %.4f) -> t_from_start=%.3f -> t_from_previous=%.3f",
            i, query_xy.x, query_xy.y, matched_time, delta_time);
    }

    moveit_msgs::msg::RobotTrajectory msg;
    robot_traj.getRobotTrajectoryMsg(msg);

    UnknownGate gate(node);

    // === Εκτέλεση με stop/resume ===
    RCLCPP_INFO(node->get_logger(), "Executing FK-time-mapped trajectory...");
    execute_with_pause_resume(move_group, msg, gate, planning_group);

    ////////////////////////////////////////////////////////////////////////////////////////


    std::string sdf_content = generate_multi_box_sdf(cubes, std::abs(cube_size_x), std::abs(cube_size_y));
    std::ofstream out("/tmp/multi_cubes.sdf");
    out << sdf_content;
    out.close();

    std::ostringstream cmd;
    cmd << "ros2 run gazebo_ros spawn_entity.py -file /tmp/multi_cubes.sdf -entity all_cubes";
    std::system(cmd.str().c_str());


    executor->cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
