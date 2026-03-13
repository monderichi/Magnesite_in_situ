#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class YBounceCmdVelSpawn : public rclcpp::Node
{
public:
  YBounceCmdVelSpawn() : rclcpp::Node("y_bounce_cmd_vel_spawn")
  {
    // ---- Resolve default SDF path (hardcoded then package share fallback) ----
    std::filesystem::path default_sdf{"/ros2_ws/src/spraying_pathways/models/human_arm/model.sdf"};
    if (!std::filesystem::exists(default_sdf)) {
      try {
        auto share = ament_index_cpp::get_package_share_directory("spraying_pathways");
        auto cand  = std::filesystem::path(share) / "models" / "human_arm" / "model.sdf";
        if (std::filesystem::exists(cand)) default_sdf = cand;
      } catch (...) {}
    }

    // ---- Parameters ----
    topic_      = this->declare_parameter<std::string>("topic", "/human_arm/cmd_vel");
    model_name_ = this->declare_parameter<std::string>("model_name", "human_arm");
    sdf_path_   = this->declare_parameter<std::string>("sdf_path", default_sdf.string());

    start_x_ = this->declare_parameter<double>("start_x", 2.26);
    start_y_ = this->declare_parameter<double>("start_y", 3.0);
    start_z_ = this->declare_parameter<double>("start_z", 1.05);
    end_y_   = this->declare_parameter<double>("end_y", -3.0);
    speed_   = this->declare_parameter<double>("speed", 0.2);     // m/s
    hz_      = this->declare_parameter<double>("update_rate", 20.0);
    pause_s_ = this->declare_parameter<double>("pause_s", 0.0);
    delete_existing_ = this->declare_parameter<bool>("delete_existing", true);

    // Derived motion values (time-based bounce)
    dist_ = std::abs(end_y_ - start_y_);
    seg_time_ = (speed_ > 1e-6 && dist_ > 1e-6) ? dist_ / speed_ : std::numeric_limits<double>::infinity();
    dir_  = (end_y_ >= start_y_) ? 1.0 : -1.0;
    phase_ = Phase::Moving;
    elapsed_ = 0.0;

    // Pub & service clients
    pub_        = this->create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
    cli_spawn_  = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    cli_delete_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    RCLCPP_INFO(get_logger(), "Will spawn '%s' from: %s",
                model_name_.c_str(), sdf_path_.c_str());

    // One-shot init timer (no blocking in ctor)
    init_timer_ = this->create_wall_timer(200ms, std::bind(&YBounceCmdVelSpawn::init_step_, this));
  }

private:
  enum class Phase { Moving, Pause };

  void init_step_()
  {
    if (!cli_spawn_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "Waiting for /spawn_entity ...");
      return;
    }
    if (delete_existing_) {
      if (!deleted_once_) {
        if (cli_delete_->service_is_ready() && !del_future_.valid()) {
          auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
          req->name = model_name_;
          del_future_ = cli_delete_->async_send_request(req);
          RCLCPP_INFO(get_logger(), "Requested delete of existing '%s' (if any).", model_name_.c_str());
          return;
        }
        if (del_future_.valid() &&
            del_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          return; // still deleting
        }
        deleted_once_ = true; // delete attempted (ignore result)
      }
    }

    if (!spawn_future_.valid()) {
      // Read SDF
      std::ifstream ifs(sdf_path_);
      if (!ifs) {
        RCLCPP_ERROR(get_logger(), "Cannot read SDF: %s", sdf_path_.c_str());
        return;
      }
      xml_.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

      auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
      req->name = model_name_;
      req->xml  = xml_;
      req->reference_frame = "world";
      req->initial_pose = geometry_msgs::msg::Pose();
      req->initial_pose.position.x = start_x_;
      req->initial_pose.position.y = start_y_;
      req->initial_pose.position.z = start_z_;
      req->initial_pose.orientation.w = 1.0;
      spawn_future_ = cli_spawn_->async_send_request(req);
      RCLCPP_INFO(get_logger(), "Spawn requested...");
      return;
    }

    if (spawn_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto res = spawn_future_.get();
      if (res && res->success) {
        RCLCPP_INFO(get_logger(), "Spawned '%s'. Starting motion on %s",
                    model_name_.c_str(), topic_.c_str());
        start_motion_();
        init_timer_->cancel();
      } else {
        std::string msg = res ? res->status_message : "no response";
        RCLCPP_WARN(get_logger(), "Spawn failed: %s. Will retry...", msg.c_str());
        // Reset for another delete+spawn cycle
        del_future_ = {};
        spawn_future_ = {};
        deleted_once_ = false;
      }
    }
  }

  void start_motion_()
  {
    if (!std::isfinite(seg_time_) || speed_ <= 1e-6 || dist_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "Speed or distance ~0; publishing zero velocity.");
    }
    const double period = 1.0 / std::max(1.0, hz_);
    move_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&YBounceCmdVelSpawn::on_tick_, this));
    RCLCPP_INFO(get_logger(),
      "Bounce params: y in [%.3f, %.3f], speed=%.3f m/s, segment T=%.3fs, pause=%.3fs",
      std::min(start_y_, end_y_), std::max(start_y_, end_y_), speed_, seg_time_, pause_s_);
  }

  void on_tick_()
  {
    // Time since last timer tick (constant by design)
    const double dt = 1.0 / std::max(1.0, hz_);
    elapsed_ += dt;

    geometry_msgs::msg::Twist cmd;

    if (phase_ == Phase::Moving) {
      cmd.linear.y = dir_ * speed_;
      if (elapsed_ >= seg_time_) {
        elapsed_ = 0.0;
        if (pause_s_ > 0.0) {
          phase_ = Phase::Pause;
          cmd = geometry_msgs::msg::Twist(); // stop during pause
        } else {
          dir_ *= -1.0; // flip immediately
        }
      }
    } else { // Pause
      // publish zero
      if (elapsed_ >= pause_s_) {
        elapsed_ = 0.0;
        phase_ = Phase::Moving;
        dir_ *= -1.0;
      }
    }

    pub_->publish(cmd);
  }

  // Params & state
  std::string topic_, model_name_, sdf_path_, xml_;
  double start_x_{}, start_y_{}, start_z_{}, end_y_{}, speed_{}, hz_{}, pause_s_{};
  double dist_{}, seg_time_{}, dir_{};
  enum Phase phase_;
  double elapsed_{};
  bool delete_existing_{}, deleted_once_{false};

  // ROS entities
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr  cli_spawn_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr cli_delete_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture del_future_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture  spawn_future_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YBounceCmdVelSpawn>());
  rclcpp::shutdown();
  return 0;
}
