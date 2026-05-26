// src/self_filter.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/point_containment_filter/shape_mask.h>  // note: .h
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometric_shapes/shapes.h>

#include <unordered_map>
#include <limits>
#include <vector>
#include <chrono>

using point_containment_filter::ShapeMask;
using point_containment_filter::ShapeHandle;
using namespace std::chrono;

class SelfFilterNode : public rclcpp::Node {
public:
  SelfFilterNode() : Node("self_filter") {
    // Parameters
    padding_        = declare_parameter<double>("self_mask_padding", 0.01);  // m
    scale_          = declare_parameter<double>("self_mask_scale",   1.0);
    input_topic_    = declare_parameter<std::string>("input_cloud",  "/final_points");
    output_topic_   = declare_parameter<std::string>("output_cloud", "/points_no_robot");
    log_every_n_    = declare_parameter<int>("log_every_n", 10);     // INFO every N clouds
    tf_cache_sec_   = declare_parameter<double>("tf_cache_sec", 60.0);
    use_latest_tf_  = declare_parameter<bool>("tf_fallback_to_latest", true);
    markers_topic_  = declare_parameter<std::string>("markers_topic", "/self_filter/collision_markers");
    marker_alpha_   = declare_parameter<double>("marker_alpha", 0.35);
    marker_scale_   = declare_parameter<double>("mesh_marker_scale", 1.0);   // only for TRIANGLE_LIST visual size (keep 1.0)

    // TF (extended cache helps with slow/laggy sim)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(tf_cache_sec_));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(),
      "SelfFilterNode: padding=%.3f scale=%.3f input='%s' output='%s' log_every_n=%d tf_cache=%.1fs fallback_latest=%s",
      padding_, scale_, input_topic_.c_str(), output_topic_.c_str(), log_every_n_,
      tf_cache_sec_, use_latest_tf_ ? "true" : "false");
  }

  bool init() {
    try {
      // RobotModelLoader -> RobotModel
      rml_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
      model_ = rml_->getModel();
      if (!model_) {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model from 'robot_description'.");
        return false;
      }
      model_frame_ = model_->getModelFrame();
      RCLCPP_INFO(get_logger(), "Loaded robot model. model_frame='%s'", model_frame_.c_str());

      // PlanningSceneMonitor (same source RViz MotionPlanning uses)
      psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), rml_, "psm");
      if (!psm_ || !psm_->getPlanningScene()) {
        RCLCPP_ERROR(get_logger(), "Failed to create PlanningSceneMonitor.");
        return false;
      }
      psm_->startStateMonitor();
      psm_->startSceneMonitor("monitored_planning_scene");
      RCLCPP_INFO(get_logger(), "PlanningSceneMonitor started (topic 'monitored_planning_scene').");

      // Build ShapeMask + remember each shape for visualization
      shape_mask_ = std::make_unique<ShapeMask>();
      shape_mask_->setTransformCallback(
        [this](ShapeHandle h, Eigen::Isometry3d& tf) { return this->shapeTransform(h, tf); });

      const auto& link_models = model_->getLinkModelsWithCollisionGeometry();
      size_t total_shapes = 0;
      for (const auto* link : link_models) {
        const auto& shapes  = link->getShapes();
        const auto& origins = link->getCollisionOriginTransforms();
        total_shapes += shapes.size();
        for (size_t i = 0; i < shapes.size(); ++i) {
          ShapeHandle h = shape_mask_->addShape(shapes[i], scale_, padding_);
          handle_map_[h] = { link->getName(), origins[i], shapes[i] };
        }
      }
      RCLCPP_INFO(get_logger(), "ShapeMask ready. links_with_collision=%zu shapes=%zu",
                  link_models.size(), total_shapes);

      // Publishers & subscriber (RELIABLE; RViz-friendly)
      pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

      sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&SelfFilterNode::cloudCb, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(),
        "I/O: subscribing '%s' [Reliable], publishing '%s' [Reliable], markers '%s'",
        input_topic_.c_str(), output_topic_.c_str(), markers_topic_.c_str());
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "init() exception: %s", e.what());
      return false;
    }
  }

private:
  struct HandleInfo {
    std::string                link_name;
    Eigen::Isometry3d          collision_origin; // shape offset in the link frame
    shapes::ShapeConstPtr      shape;            // for visualization
  };

  // ----- small helpers -----
  static geometry_msgs::msg::Pose isoToPose(const Eigen::Isometry3d& T) {
    geometry_msgs::msg::Pose p;
    const auto& t = T.translation();
    const Eigen::Quaterniond q(T.rotation());
    p.position.x = t.x(); p.position.y = t.y(); p.position.z = t.z();
    p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
    return p;
  }

  visualization_msgs::msg::Marker makeMarkerFromShape(
      const shapes::Shape& shape,
      const Eigen::Isometry3d& T_target_shape,
      int id,
      const std::string& frame) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp    = now();
    m.ns = "self_filter_collision";
    m.id = id;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = isoToPose(T_target_shape);
    m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f; m.color.a = static_cast<float>(marker_alpha_);
    m.frame_locked = true;

    switch (shape.type) {
      case shapes::ShapeType::BOX: {
        const auto& b = static_cast<const shapes::Box&>(shape);
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = b.size[0]; m.scale.y = b.size[1]; m.scale.z = b.size[2];
        break;
      }
      case shapes::ShapeType::CYLINDER: {
        const auto& c = static_cast<const shapes::Cylinder&>(shape);
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 2.0 * c.radius;  // diameter
        m.scale.y = 2.0 * c.radius;
        m.scale.z = c.length;
        break;
      }
      case shapes::ShapeType::SPHERE: {
        const auto& s = static_cast<const shapes::Sphere&>(shape);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = m.scale.y = m.scale.z = 2.0 * s.radius;
        break;
      }
      case shapes::ShapeType::CONE: {
        // Approximate with a cylinder (good enough for debugging)
        const auto& cn = static_cast<const shapes::Cone&>(shape);
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 2.0 * cn.radius;
        m.scale.y = 2.0 * cn.radius;
        m.scale.z = cn.length;
        break;
      }
      case shapes::ShapeType::MESH: {
        const auto& mesh = static_cast<const shapes::Mesh&>(shape);
        m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        m.scale.x = m.scale.y = m.scale.z = marker_scale_;
        m.points.reserve(mesh.triangle_count * 3);
        for (unsigned int t = 0; t < mesh.triangle_count; ++t) {
          const unsigned int i0 = mesh.triangles[3 * t + 0];
          const unsigned int i1 = mesh.triangles[3 * t + 1];
          const unsigned int i2 = mesh.triangles[3 * t + 2];
          geometry_msgs::msg::Point p0, p1, p2;
          p0.x = mesh.vertices[3 * i0 + 0]; p0.y = mesh.vertices[3 * i0 + 1]; p0.z = mesh.vertices[3 * i0 + 2];
          p1.x = mesh.vertices[3 * i1 + 0]; p1.y = mesh.vertices[3 * i1 + 1]; p1.z = mesh.vertices[3 * i1 + 2];
          p2.x = mesh.vertices[3 * i2 + 0]; p2.y = mesh.vertices[3 * i2 + 1]; p2.z = mesh.vertices[3 * i2 + 2];
          m.points.push_back(p0); m.points.push_back(p1); m.points.push_back(p2);
        }
        break;
      }
      default: { // plane or unknown -> draw a small cube
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = m.scale.y = m.scale.z = 0.02;
        break;
      }
    }
    return m;
  }

  // ShapeMask transform callback:
  //   out_tf (shape in cloud frame) = T_target_model_ * T_model_link * collision_origin
  bool shapeTransform(ShapeHandle h, Eigen::Isometry3d& out_tf) {
    auto it = handle_map_.find(h);
    if (it == handle_map_.end() || !state_snapshot_) return false;

    const auto& info = it->second;
    const Eigen::Isometry3d T_model_link = state_snapshot_->getGlobalLinkTransform(info.link_name);
    out_tf = T_target_model_ * T_model_link * info.collision_origin;
    return true;
  }

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto t0 = steady_clock::now();

    // 1) Received
    target_frame_ = msg->header.frame_id;
    last_stamp_   = msg->header.stamp;

    const uint32_t num_pts = msg->width * msg->height;
    if (num_pts == 0) {
      RCLCPP_WARN(get_logger(), "Received EMPTY cloud (w=%u h=%u). Skipping.", msg->width, msg->height);
      return;
    }
    ++cloud_count_;
    RCLCPP_DEBUG(get_logger(), "Cloud #%zu received: frame='%s' points=%u (w=%u h=%u)",
                 cloud_count_, target_frame_.c_str(), num_pts, msg->width, msg->height);

    // 2) Snapshot of RobotState from PlanningScene
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
      state_snapshot_ = std::make_unique<moveit::core::RobotState>(scene->getCurrentState());
    }

    // 3) One TF: cloud frame -> model frame (use latest fallback if needed)
    try {
      auto st = tf_buffer_->lookupTransform(model_frame_, target_frame_, last_stamp_);
      // st is T_model_target; store its inverse (T_target_model_)
      T_target_model_ = tf2::transformToEigen(st.transform).inverse();
    } catch (const tf2::TransformException& ex) {
      if (use_latest_tf_) {
        RCLCPP_WARN(get_logger(),
          "TF(model<-cloud) at stamp failed (%s). Falling back to LATEST.", ex.what());
        auto st = tf_buffer_->lookupTransform(model_frame_, target_frame_, rclcpp::Time(0));
        T_target_model_ = tf2::transformToEigen(st.transform).inverse();
      } else {
        RCLCPP_WARN(get_logger(),
          "TF(model<-cloud) at stamp failed and fallback disabled: %s", ex.what());
        return;
      }
    }

    // 4) Mask (0=INSIDE, 1=OUTSIDE, 2=CLIP)
    std::vector<int> mask;
    Eigen::Vector3d sensor_origin(0,0,0);
    shape_mask_->maskContainment(*msg, sensor_origin,
                                 /*min=*/0.0,
                                 /*max=*/std::numeric_limits<double>::infinity(),
                                 mask);
    if (mask.size() != num_pts) {
      RCLCPP_WARN(get_logger(), "Mask size mismatch: mask=%zu vs points=%u", mask.size(), num_pts);
      return;
    }

    // 5) Filter (copy OUTSIDE)
    sensor_msgs::msg::PointCloud2 out;
    out.header       = msg->header;
    out.height       = 1;
    out.is_bigendian = msg->is_bigendian;
    out.fields       = msg->fields;
    out.point_step   = msg->point_step;
    out.is_dense     = false;

    out.data.reserve(msg->data.size());
    const auto step = msg->point_step;
    size_t kept = 0;
    for (size_t i = 0; i < num_pts; ++i) {
      if (mask[i] == 1) {
        const uint8_t* src = &msg->data[i * step];
        out.data.insert(out.data.end(), src, src + step);
        ++kept;
      }
    }
    out.width    = static_cast<uint32_t>(kept);
    out.row_step = out.width * out.point_step;

    total_points_in_  += num_pts;
    total_points_out_ += kept;

    const auto dt_ms  = duration_cast<milliseconds>(steady_clock::now() - t0).count();
    const double keep = (num_pts > 0) ? 100.0 * kept / static_cast<double>(num_pts) : 0.0;

    // 6) Publish filtered cloud
    pub_->publish(out);

    // 7) Publish collision markers (in the cloud frame)
    if (marker_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray arr;
      int id = 0;
      for (const auto& kv : handle_map_) {
        const auto& info = kv.second;
        const Eigen::Isometry3d T_model_link = state_snapshot_->getGlobalLinkTransform(info.link_name);
        const Eigen::Isometry3d T_target_shape = T_target_model_ * T_model_link * info.collision_origin;
        arr.markers.push_back(makeMarkerFromShape(*info.shape, T_target_shape, id++, target_frame_));
      }
      marker_pub_->publish(arr);
    }

    // 8) Logs
    if (cloud_count_ <= 3 || (log_every_n_ > 0 && (cloud_count_ % static_cast<size_t>(log_every_n_) == 0))) {
      RCLCPP_INFO(get_logger(),
        "Cloud #%zu: in=%u out=%zu (kept=%.1f%%) time=%ldms subs=%zu",
        cloud_count_, num_pts, kept, keep, dt_ms, pub_->get_subscription_count());
    } else {
      RCLCPP_DEBUG(get_logger(),
        "Cloud #%zu: in=%u out=%zu (%.1f%%) time=%ldms subs=%zu",
        cloud_count_, num_pts, kept, keep, dt_ms, pub_->get_subscription_count());
    }
  }

  // Params
  double      padding_{0.01}, scale_{1.0};
  std::string input_topic_, output_topic_;
  int         log_every_n_{10};
  double      tf_cache_sec_{60.0};
  bool        use_latest_tf_{true};
  std::string markers_topic_;
  double      marker_alpha_{0.35};
  double      marker_scale_{1.0};

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // MoveIt / masking
  robot_model_loader::RobotModelLoaderPtr      rml_;
  moveit::core::RobotModelPtr                  model_;
  std::unique_ptr<ShapeMask>                   shape_mask_;
  struct HandleInfo;
  std::unordered_map<ShapeHandle, HandleInfo>  handle_map_;

  // PlanningSceneMonitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::unique_ptr<moveit::core::RobotState>       state_snapshot_;
  std::string                                      model_frame_;
  Eigen::Isometry3d                                 T_target_model_{Eigen::Isometry3d::Identity()};

  // Per-cloud context / totals
  std::string target_frame_;
  rclcpp::Time last_stamp_;
  size_t cloud_count_{0};
  uint64_t total_points_in_{0};
  uint64_t total_points_out_{0};

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher  <sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher  <visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfFilterNode>();
  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "SelfFilterNode init() failed.");
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
