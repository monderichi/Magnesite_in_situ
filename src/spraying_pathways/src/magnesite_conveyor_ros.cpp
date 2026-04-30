/**
 * Magnesite Conveyor ROS2 Node (C++)
 *
 * C++ port of magnesite_conveyor_ros.py
 * - YOLOv8-seg inference via ONNX Runtime
 * - LAB + OTSU magnesite classification
 * - Simple IOU-based multi-object tracking
 * - Trigger / verify line zones with 3D point publishing
 * - Interactive OpenCV visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <onnxruntime_cxx_api.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

// ============================================================
// Configuration
// ============================================================
static const std::string MODEL_PATH = "/media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/yolo_uv/only_rock.onnx";
static const float MAGNESITE_THRESHOLD = 50.0f;
static const float CONFIDENCE = 0.35f;
static const int INFERENCE_SIZE = 640;
static const int MIN_MASK_PIXELS = 200;
static const cv::Size BLUR_KERNEL(5, 5);
static const int SKIP_FRAMES = 1;
static const float ZOOM_FACTOR = 2.0f;
static const int DISPLAY_W = 1280;
static const int DISPLAY_H = 800;

// ============================================================
// Helper: Center Zoom
// ============================================================
static cv::Mat center_zoom(const cv::Mat& frame, float zoom, cv::Rect& crop_rect)
{
  int h = frame.rows;
  int w = frame.cols;
  int crop_w = static_cast<int>(w / zoom);
  int crop_h = static_cast<int>(h / zoom);
  int x1 = (w - crop_w) / 2;
  int y1 = (h - crop_h) / 2;
  crop_rect = cv::Rect(x1, y1, crop_w, crop_h);
  cv::Mat cropped = frame(cv::Rect(x1, y1, crop_w, crop_h));
  cv::Mat zoomed;
  cv::resize(cropped, zoomed, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
  return zoomed;
}

static cv::Point2f zoomed_pixel_to_original(float zx, float zy, const cv::Rect& crop_rect, int frame_w, int frame_h)
{
  int crop_w = crop_rect.width;
  int crop_h = crop_rect.height;
  float orig_x = (zx / frame_w) * crop_w + crop_rect.x;
  float orig_y = (zy / frame_h) * crop_h + crop_rect.y;
  return cv::Point2f(orig_x, orig_y);
}

// ============================================================
// Helper: LAB + OTSU Classification
// ============================================================
static bool classify_rock_lab_otsu(const cv::Mat& frame_bgr, const cv::Mat& rock_mask,
                                   float& white_pct, float& dark_pct, float& ratio, float& thresh_val)
{
  int total_pixels = cv::countNonZero(rock_mask);
  if (total_pixels < MIN_MASK_PIXELS) {
    white_pct = dark_pct = ratio = thresh_val = 0.0f;
    return false;
  }

  cv::Rect bbox = cv::boundingRect(rock_mask);
  cv::Mat roi_frame = frame_bgr(bbox);
  cv::Mat roi_mask = rock_mask(bbox);

  cv::Mat lab_roi;
  cv::cvtColor(roi_frame, lab_roi, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> lab_channels;
  cv::split(lab_roi, lab_channels);
  cv::Mat l_blurred;
  cv::GaussianBlur(lab_channels[0], l_blurred, BLUR_KERNEL, 0.0);

  cv::Mat binary;
  double thresh = cv::threshold(l_blurred, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  thresh_val = static_cast<float>(thresh);

  cv::Mat white_mask, dark_mask;
  cv::bitwise_and(binary, roi_mask, white_mask);
  cv::Mat inverted_binary = ~binary;
  cv::bitwise_and(inverted_binary, roi_mask, dark_mask);

  int white_count = cv::countNonZero(white_mask);
  int dark_count = cv::countNonZero(dark_mask);

  white_pct = (white_count / static_cast<float>(total_pixels)) * 100.0f;
  dark_pct = (dark_count / static_cast<float>(total_pixels)) * 100.0f;
  ratio = (dark_count > 0) ? (white_count / static_cast<float>(dark_count)) : std::numeric_limits<float>::infinity();

  return white_pct > MAGNESITE_THRESHOLD;
}

// ============================================================
// ONNX YOLO Segmentation Inferencer
// ============================================================
class OnnxYoloSeg
{
public:
  struct Detection
  {
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float confidence = 0;
    int class_id = 0;
    cv::Mat mask; // binary mask (uint8, 0/255), sized to detection bbox in zoomed frame coords
  };

  explicit OnnxYoloSeg(const std::string& model_path, float conf_thresh = 0.35f)
    : env_(ORT_LOGGING_LEVEL_WARNING, "YOLO_SEG"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      conf_thresh_(conf_thresh)
  {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(static_cast<int>(std::thread::hardware_concurrency()));
    opts.SetInterOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

    // Verify input shape
    Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = input_tensor_info.GetShape();
    if (input_shape_.size() == 4) {
      input_h_ = static_cast<int>(input_shape_[2]);
      input_w_ = static_cast<int>(input_shape_[3]);
    }

    // Output names
    Ort::AllocatorWithDefaultOptions allocator;
    {
      auto alloc_name = session_->GetInputNameAllocated(0, allocator);
      input_name_str_ = alloc_name.get();
      input_name_ptr_ = input_name_str_.c_str();
    }
    size_t out_count = session_->GetOutputCount();
    for (size_t i = 0; i < out_count; ++i) {
      auto alloc_name = session_->GetOutputNameAllocated(i, allocator);
      output_name_strs_.push_back(alloc_name.get());
    }
    for (const auto& str : output_name_strs_) {
      output_name_ptrs_.push_back(str.c_str());
    }
  }

  std::vector<Detection> infer(const cv::Mat& bgr_image)
  {
    int orig_h = bgr_image.rows;
    int orig_w = bgr_image.cols;

    std::vector<float> input_tensor = preprocess(bgr_image);
    std::vector<int64_t> input_dims = {1, 3, input_h_, input_w_};
    Ort::Value input_ort = Ort::Value::CreateTensor<float>(
      memory_info_, input_tensor.data(), input_tensor.size(), input_dims.data(), input_dims.size());

    Ort::RunOptions run_options;
    std::vector<Ort::Value> outputs = session_->Run(
      run_options, &input_name_ptr_, &input_ort, 1,
      output_name_ptrs_.data(), output_name_ptrs_.size());

    return postprocess(outputs, orig_w, orig_h);
  }

private:
  Ort::Env env_;
  Ort::MemoryInfo memory_info_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_str_;
  const char* input_name_ptr_ = nullptr;
  std::vector<std::string> output_name_strs_;
  std::vector<const char*> output_name_ptrs_;
  std::vector<int64_t> input_shape_;
  float conf_thresh_;
  int input_w_ = 640;
  int input_h_ = 640;

  std::vector<float> preprocess(const cv::Mat& bgr)
  {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);

    cv::Mat float_img;
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // HWC -> CHW
    std::vector<cv::Mat> chw(3);
    cv::split(float_img, chw);
    std::vector<float> input_data;
    input_data.reserve(3 * input_w_ * input_h_);
    for (int c = 0; c < 3; ++c) {
      input_data.insert(input_data.end(), (float*)chw[c].datastart, (float*)chw[c].dataend);
    }
    return input_data;
  }

  std::vector<Detection> postprocess(const std::vector<Ort::Value>& outputs, int orig_w, int orig_h)
  {
    // outputs[0]: [1, 300, 38]  -> boxes + scores + class + mask_coeffs
    // outputs[1]: [1, 32, 160, 160] -> mask prototypes

    const float* out0 = outputs[0].GetTensorData<float>();
    auto shape0 = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_dets = static_cast<int>(shape0[1]);
    int feat_len = static_cast<int>(shape0[2]); // should be 38

    const float* out1 = outputs[1].GetTensorData<float>();
    auto shape1 = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
    int num_protos = static_cast<int>(shape1[1]); // 32
    int proto_h = static_cast<int>(shape1[2]);    // 160
    int proto_w = static_cast<int>(shape1[3]);    // 160
    int proto_area = proto_h * proto_w;

    // Build prototype matrix ONCE per frame: [num_protos, proto_area] = [32, 25600]
    // This avoids re-wrapping the pointer every detection.
    cv::Mat proto_mat(num_protos, proto_area, CV_32FC1, const_cast<float*>(out1));

    std::vector<Detection> detections;

    float scale_x = orig_w / static_cast<float>(input_w_);
    float scale_y = orig_h / static_cast<float>(input_h_);

    for (int i = 0; i < num_dets; ++i) {
      const float* row = out0 + i * feat_len;
      float score = row[4];
      if (score < conf_thresh_) continue;

      float x1 = row[0] * scale_x;
      float y1 = row[1] * scale_y;
      float x2 = row[2] * scale_x;
      float y2 = row[3] * scale_y;

      // Clamp
      x1 = std::max(0.0f, std::min(x1, static_cast<float>(orig_w - 1)));
      y1 = std::max(0.0f, std::min(y1, static_cast<float>(orig_h - 1)));
      x2 = std::max(0.0f, std::min(x2, static_cast<float>(orig_w - 1)));
      y2 = std::max(0.0f, std::min(y2, static_cast<float>(orig_h - 1)));

      if (x2 <= x1 || y2 <= y1) continue;

      Detection det;
      det.x1 = x1; det.y1 = y1; det.x2 = x2; det.y2 = y2;
      det.confidence = score;
      det.class_id = static_cast<int>(row[5]);

      // --- Fast mask decode via matrix multiply ---
      // coeffs: [1, 32],  proto_mat: [32, 25600]  ->  result: [1, 25600]
      // Equivalent to numpy: (coeffs @ protos).reshape(160,160)
      const float* coeffs = row + 6; // 32 coefficients
      cv::Mat coeff_row(1, num_protos, CV_32FC1, const_cast<float*>(coeffs));
      cv::Mat mask_flat; // [1, proto_area]
      cv::gemm(coeff_row, proto_mat, 1.0, cv::Mat(), 0.0, mask_flat);

      // Reshape to [proto_h, proto_w] and apply sigmoid via vectorized exp()
      cv::Mat mask_160 = mask_flat.reshape(1, proto_h);
      cv::Mat neg_mask;
      cv::exp(-mask_160, neg_mask);           // exp(-x)
      cv::Mat mask_sig = 1.0f / (1.0f + neg_mask); // sigmoid

      // Resize directly 160x160 -> orig size (skip the pointless 640 intermediate)
      cv::Mat mask_orig;
      cv::resize(mask_sig, mask_orig, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);

      // Crop to bbox and threshold
      int bx1 = static_cast<int>(x1);
      int by1 = static_cast<int>(y1);
      int bx2 = static_cast<int>(x2);
      int by2 = static_cast<int>(y2);
      cv::Mat mask_crop = mask_orig(cv::Rect(bx1, by1, bx2 - bx1, by2 - by1));
      cv::Mat mask_bin;
      cv::threshold(mask_crop, mask_bin, 0.5, 255, cv::THRESH_BINARY);
      mask_bin.convertTo(det.mask, CV_8UC1);

      detections.push_back(det);
    }

    return detections;
  }
};

// ============================================================
// Simple IOU Tracker
// ============================================================
class SimpleTracker
{
public:
  struct Track
  {
    int id = -1;
    int class_id = 0;
    cv::Rect2f bbox;
    cv::Point2f center;
    cv::Point2f prev_center;
    int age = 0;
    int hits = 0;
    bool active = true;
    bool is_magnesite = false;
    bool triggered = false;
    bool verified = false;
    cv::Mat mask;
  };

  explicit SimpleTracker(float iou_thresh = 0.3f, int max_age = 10)
    : iou_thresh_(iou_thresh), max_age_(max_age) {}

  std::vector<Track> update(const std::vector<OnnxYoloSeg::Detection>& detections)
  {
    // Predict / age tracks
    for (auto& t : tracks_) {
      t.prev_center = t.center;
      t.age++;
    }

    // Build cost matrix (1 - IOU)
    size_t n_tracks = tracks_.size();
    size_t n_dets = detections.size();
    std::vector<std::vector<float>> cost(n_tracks, std::vector<float>(n_dets, 1.0f));

    for (size_t i = 0; i < n_tracks; ++i) {
      for (size_t j = 0; j < n_dets; ++j) {
        float iou_val = iou(tracks_[i].bbox, cv::Rect2f(detections[j].x1, detections[j].y1,
                                                          detections[j].x2 - detections[j].x1,
                                                          detections[j].y2 - detections[j].y1));
        cost[i][j] = 1.0f - iou_val;
      }
    }

    // Greedy assignment
    std::vector<bool> track_matched(n_tracks, false);
    std::vector<bool> det_matched(n_dets, false);
    std::vector<std::pair<size_t, size_t>> matches;

    while (true) {
      float min_cost = 1.0f;
      size_t min_i = 0, min_j = 0;
      bool found = false;
      for (size_t i = 0; i < n_tracks; ++i) {
        if (track_matched[i]) continue;
        for (size_t j = 0; j < n_dets; ++j) {
          if (det_matched[j]) continue;
          if (cost[i][j] < min_cost) {
            min_cost = cost[i][j];
            min_i = i;
            min_j = j;
            found = true;
          }
        }
      }
      if (!found || min_cost > (1.0f - iou_thresh_)) break;
      matches.emplace_back(min_i, min_j);
      track_matched[min_i] = true;
      det_matched[min_j] = true;
    }

    // Update matched
    for (auto& [ti, di] : matches) {
      auto& t = tracks_[ti];
      const auto& d = detections[di];
      t.bbox = cv::Rect2f(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1);
      t.center = cv::Point2f((d.x1 + d.x2) / 2.0f, (d.y1 + d.y2) / 2.0f);
      t.class_id = d.class_id;
      t.age = 0;
      t.hits++;
      t.active = true;
      t.mask = d.mask.clone();
    }

    // Unmatched tracks -> mark inactive
    for (size_t i = 0; i < n_tracks; ++i) {
      if (!track_matched[i]) {
        tracks_[i].active = false;
      }
    }

    // Unmatched detections -> new tracks
    for (size_t j = 0; j < n_dets; ++j) {
      if (!det_matched[j]) {
        Track t;
        t.id = next_id_++;
        const auto& d = detections[j];
        t.bbox = cv::Rect2f(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1);
        t.center = cv::Point2f((d.x1 + d.x2) / 2.0f, (d.y1 + d.y2) / 2.0f);
        t.prev_center = t.center;
        t.class_id = d.class_id;
        t.age = 0;
        t.hits = 1;
        t.active = true;
        t.mask = d.mask.clone();
        tracks_.push_back(t);
      }
    }

    // Remove old dead tracks
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const Track& t) { return !t.active && t.age > max_age_; }),
      tracks_.end());

    return tracks_;
  }

private:
  float iou_thresh_;
  int max_age_;
  int next_id_ = 1;
  std::vector<Track> tracks_;

  static float iou(const cv::Rect2f& a, const cv::Rect2f& b)
  {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);
    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area_a = a.width * a.height;
    float area_b = b.width * b.height;
    float uni = area_a + area_b - inter;
    return (uni > 0.0f) ? (inter / uni) : 0.0f;
  }
};

// ============================================================
// Line Crossing Helper
// ============================================================
static bool segments_intersect(const cv::Point2f& p1, const cv::Point2f& p2,
                               const cv::Point2f& a, const cv::Point2f& b)
{
  auto ccw = [](const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C) -> bool {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  };
  return (ccw(p1, a, b) != ccw(p2, a, b)) && (ccw(p1, p2, a) != ccw(p1, p2, b));
}

// ============================================================
// Main ROS2 Node
// ============================================================
class MagnesiteConveyorNode : public rclcpp::Node
{
public:
  MagnesiteConveyorNode()
    : Node("magnesite_conveyor_ros_cpp"),
      inferencer_(MODEL_PATH, CONFIDENCE),
      tracker_(0.3f, 10),
      zoom_(ZOOM_FACTOR),
      inference_size_(INFERENCE_SIZE),
      skip_frames_(SKIP_FRAMES),
      frame_count_(0),
      drawing_mode_(DrawMode::NONE)
  {
    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/magnesite_target", 10);
    alert_pub_ = this->create_publisher<std_msgs::msg::String>("/magnesite_alerts", 10);

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/aligned_depth_to_color/camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        cam_model_.fromCameraInfo(*msg);
        intrinsics_loaded_ = true;
        cam_info_sub_.reset(); // unsubscribe after first message
      });

    sub_color_.subscribe(this, "/camera/camera/color/image_raw");
    sub_depth_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_color_, sub_depth_);
    sync_->registerCallback(&MagnesiteConveyorNode::sync_callback, this);

    trigger_start_ = cv::Point2f(50.0f, 400.0f);
    trigger_end_ = cv::Point2f(DISPLAY_W - 50.0f, 400.0f);
    verify_start_ = cv::Point2f(50.0f, 700.0f);
    verify_end_ = cv::Point2f(DISPLAY_W - 50.0f, 700.0f);

    cv::namedWindow("Magnesite Conveyor Pipeline (C++)", cv::WINDOW_NORMAL);
    cv::resizeWindow("Magnesite Conveyor Pipeline (C++)", DISPLAY_W, DISPLAY_H);
    cv::setMouseCallback("Magnesite Conveyor Pipeline (C++)", mouse_callback_static, this);

    RCLCPP_INFO(this->get_logger(), "C++ CV Node Initialized. Waiting for camera topics...");
  }

  ~MagnesiteConveyorNode()
  {
    cv::destroyAllWindows();
  }

private:
  enum class DrawMode { NONE, TRIGGER, VERIFY };

  OnnxYoloSeg inferencer_;
  SimpleTracker tracker_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> sub_color_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  image_geometry::PinholeCameraModel cam_model_;
  bool intrinsics_loaded_ = false;

  float zoom_;
  int inference_size_;
  int skip_frames_;
  int frame_count_;

  cv::Point2f trigger_start_, trigger_end_;
  cv::Point2f verify_start_, verify_end_;

  DrawMode drawing_mode_;
  std::vector<cv::Point> current_clicks_;

  cv::TickMeter fps_meter_;
  double inference_ms_ = 0.0;

  static void mouse_callback_static(int event, int x, int y, int flags, void* userdata)
  {
    auto* self = static_cast<MagnesiteConveyorNode*>(userdata);
    self->mouse_callback(event, x, y, flags);
  }

  void mouse_callback(int event, int x, int y, int flags)
  {
    (void)flags;
    if (event == cv::EVENT_LBUTTONDOWN && drawing_mode_ != DrawMode::NONE) {
      current_clicks_.push_back(cv::Point(x, y));
      if (current_clicks_.size() == 2) {
        cv::Point2f p1(current_clicks_[0].x, current_clicks_[0].y);
        cv::Point2f p2(current_clicks_[1].x, current_clicks_[1].y);
        if (drawing_mode_ == DrawMode::TRIGGER) {
          trigger_start_ = p1;
          trigger_end_ = p2;
          RCLCPP_INFO(this->get_logger(), "Trigger Line updated: (%.0f,%.0f) -> (%.0f,%.0f)", p1.x, p1.y, p2.x, p2.y);
        } else if (drawing_mode_ == DrawMode::VERIFY) {
          verify_start_ = p1;
          verify_end_ = p2;
          RCLCPP_INFO(this->get_logger(), "Verify Line updated: (%.0f,%.0f) -> (%.0f,%.0f)", p1.x, p1.y, p2.x, p2.y);
        }
        drawing_mode_ = DrawMode::NONE;
        current_clicks_.clear();
      }
    }
  }

  float get_valid_depth(const cv::Mat& depth_img, int x, int y, int max_radius = 5)
  {
    int h = depth_img.rows;
    int w = depth_img.cols;
    if (x >= 0 && x < w && y >= 0 && y < h) {
      float d = depth_img.at<uint16_t>(y, x);
      if (d > 0) return d;
    }
    for (int r = 1; r <= max_radius; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
            float d = depth_img.at<uint16_t>(ny, nx);
            if (d > 0) return d;
          }
        }
      }
    }
    return 0.0f;
  }

  void publish_target(float x, float y, float z, int tracker_id)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_color_optical_frame";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;
    target_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published target ID %d: X=%.3f, Y=%.3f, Z=%.3f", tracker_id, x, y, z);
  }

  void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    if (!intrinsics_loaded_) return;

    fps_meter_.start();
    frame_count_++;

    cv::Mat full_frame = cv_bridge::toCvCopy(color_msg, "bgr8")->image;
    cv::Mat depth_img = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    // Resize to display resolution
    cv::resize(full_frame, full_frame, cv::Size(DISPLAY_W, DISPLAY_H));

    // Center zoom
    cv::Rect crop_rect;
    cv::Mat zoomed_frame = center_zoom(full_frame, zoom_, crop_rect);

    bool run_inference = (frame_count_ % skip_frames_ == 0) || last_tracks_.empty();
    std::vector<SimpleTracker::Track> tracks;

    if (run_inference) {
      auto t0 = std::chrono::high_resolution_clock::now();
      auto detections = inferencer_.infer(zoomed_frame);
      auto t1 = std::chrono::high_resolution_clock::now();
      inference_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

      // LAB + OTSU classification on each detection
      for (auto& det : detections) {
        float white_pct = 0, dark_pct = 0, ratio = 0, thresh = 0;
        bool is_mag = classify_rock_lab_otsu(zoomed_frame, det.mask, white_pct, dark_pct, ratio, thresh);
        // Store classification result in class_id: 1 = magnesite, 0 = rock
        det.class_id = is_mag ? 1 : 0;
      }

      tracks = tracker_.update(detections);
      last_tracks_ = tracks;
    } else {
      tracks = last_tracks_;
      inference_ms_ = 0.0;
    }

    // Trigger / Verify zones
    for (auto& track : tracks) {
      if (!track.active || track.hits < 2) continue;

      // Trigger line crossing
      if (!track.triggered) {
        if (segments_intersect(track.prev_center, track.center, trigger_start_, trigger_end_)) {
          track.triggered = true;
          if (track.is_magnesite || track.class_id == 1) {
            float cx = (track.bbox.x + track.bbox.x + track.bbox.width) / 2.0f;
            float cy = (track.bbox.y + track.bbox.y + track.bbox.height) / 2.0f;
            auto orig = zoomed_pixel_to_original(cx, cy, crop_rect, DISPLAY_W, DISPLAY_H);

            int df_h = depth_img.rows;
            int df_w = depth_img.cols;
            int native_x = static_cast<int>(orig.x * (df_w / static_cast<float>(DISPLAY_W)));
            int native_y = static_cast<int>(orig.y * (df_h / static_cast<float>(DISPLAY_H)));

            float depth_mm = get_valid_depth(depth_img, native_x, native_y);
            if (depth_mm > 0.0f) {
              float depth_m = depth_mm / 1000.0f;
              cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(native_x, native_y));
              publish_target(static_cast<float>(ray.x * depth_m),
                             static_cast<float>(ray.y * depth_m),
                             static_cast<float>(depth_m),
                             track.id);
            } else {
              RCLCPP_ERROR(this->get_logger(), "No depth at (%d, %d) for ID %d", native_x, native_y, track.id);
            }
          }
        }
      }

      // Verify line crossing
      if (!track.verified) {
        if (segments_intersect(track.prev_center, track.center, verify_start_, verify_end_)) {
          track.verified = true;
          if (track.is_magnesite || track.class_id == 1) {
            std::string alert = "ALERT! Magnesite ID " + std::to_string(track.id) + " reached Verification Line!";
            RCLCPP_WARN(this->get_logger(), "%s", alert.c_str());
            std_msgs::msg::String msg;
            msg.data = alert;
            alert_pub_->publish(msg);
          }
        }
      }
    }

    // Visualization
    cv::Mat annotated = zoomed_frame.clone();
    for (const auto& track : tracks) {
      if (!track.active) continue;

      cv::Scalar color = (track.class_id == 1) ? cv::Scalar(255, 100, 0) : cv::Scalar(0, 200, 0);
      std::string label = "ID: " + std::to_string(track.id) + ((track.class_id == 1) ? " MAG" : " ROCK");

      // Draw mask if available
      if (!track.mask.empty()) {
        cv::Rect bbox(static_cast<int>(track.bbox.x), static_cast<int>(track.bbox.y),
                      static_cast<int>(track.bbox.width), static_cast<int>(track.bbox.height));
        if (bbox.width > 0 && bbox.height > 0 &&
            bbox.x >= 0 && bbox.y >= 0 &&
            bbox.x + bbox.width <= annotated.cols && bbox.y + bbox.height <= annotated.rows) {
          cv::Mat roi = annotated(bbox);
          // Resize mask to current bbox size in case the tracker updated the bbox
          cv::Mat resized_mask;
          cv::resize(track.mask, resized_mask, cv::Size(bbox.width, bbox.height), 0, 0, cv::INTER_NEAREST);
          cv::Mat colored_mask;
          cv::cvtColor(resized_mask, colored_mask, cv::COLOR_GRAY2BGR);
          colored_mask.setTo(color, resized_mask);
          cv::addWeighted(roi, 1.0, colored_mask, 0.4, 0.0, roi);
        }
      }

      // Draw box
      cv::rectangle(annotated, cv::Point(static_cast<int>(track.bbox.x), static_cast<int>(track.bbox.y)),
                    cv::Point(static_cast<int>(track.bbox.x + track.bbox.width),
                              static_cast<int>(track.bbox.y + track.bbox.height)),
                    color, 2);

      // Draw label
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
      cv::Point text_org(track.bbox.x, std::max(static_cast<int>(track.bbox.y) - 5, text_size.height + 5));
      cv::rectangle(annotated, text_org + cv::Point(0, baseline),
                    text_org + cv::Point(text_size.width, -text_size.height), cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(annotated, label, text_org, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }

    // Draw lines
    cv::line(annotated, trigger_start_, trigger_end_, cv::Scalar(0, 0, 255), 2);
    cv::line(annotated, verify_start_, verify_end_, cv::Scalar(0, 255, 255), 2);
    cv::putText(annotated, "TRIGGER LINE",
                cv::Point(static_cast<int>(trigger_start_.x), static_cast<int>(trigger_start_.y) - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    cv::putText(annotated, "VERIFICATION LINE",
                cv::Point(static_cast<int>(verify_start_.x), static_cast<int>(verify_start_.y) - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

    // Drawing mode UI
    if (drawing_mode_ != DrawMode::NONE) {
      std::string mode_str = (drawing_mode_ == DrawMode::TRIGGER) ? "TRIGGER" : "VERIFY";
      cv::putText(annotated, "CLICK 2 POINTS FOR " + mode_str + " LINE",
                  cv::Point(DISPLAY_W / 2 - 300, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
      for (const auto& c : current_clicks_) {
        cv::circle(annotated, c, 5, cv::Scalar(255, 255, 255), -1);
      }
    }

    // Zoom text
    std::string zoom_text = cv::format("ZOOM %.1fx", zoom_);
    cv::putText(annotated, zoom_text, cv::Point(DISPLAY_W / 2 - 50, DISPLAY_H - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 220, 255), 2);

    fps_meter_.stop();

    // Info panel
    int panel_h = 130, panel_w = 520;
    cv::Rect panel_roi(5, 5, panel_w, panel_h);
    if (panel_roi.x + panel_roi.width <= annotated.cols && panel_roi.y + panel_roi.height <= annotated.rows) {
      cv::Mat roi = annotated(panel_roi);
      cv::Mat darkened = roi * 0.4;
      darkened.copyTo(roi);
    }

    cv::putText(annotated, cv::format("FPS: %.1f | Inference: %.0fms | Res: %d",
                                       fps_meter_.getFPS(), inference_ms_, inference_size_),
                cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
    cv::putText(annotated, "Press 'h' for controls | 'q' to quit",
                cv::Point(15, 91), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(150, 150, 150), 1);
    cv::putText(annotated, "Press 't' draw trigger | 'v' draw verify",
                cv::Point(15, 113), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 255, 255), 1);

    cv::imshow("Magnesite Conveyor Pipeline (C++)", annotated);
    int key = cv::waitKey(1) & 0xFF;

    if (key == 'q' || key == 27) {
      rclcpp::shutdown();
    } else if (key == 't') {
      drawing_mode_ = DrawMode::TRIGGER;
      current_clicks_.clear();
    } else if (key == 'v') {
      drawing_mode_ = DrawMode::VERIFY;
      current_clicks_.clear();
    } else if (key == '+' || key == '=') {
      zoom_ = std::min(zoom_ + 0.25f, 8.0f);
    } else if (key == '-' || key == '_') {
      zoom_ = std::max(zoom_ - 0.25f, 1.0f);
    } else if (key == 'i' || key == 'I') {
      inference_size_ = std::min(inference_size_ + 64, 1280);
      RCLCPP_INFO(this->get_logger(), "Resolution increased to %d", inference_size_);
    } else if (key == 'd' || key == 'D') {
      inference_size_ = std::max(inference_size_ - 64, 320);
      RCLCPP_INFO(this->get_logger(), "Resolution decreased to %d", inference_size_);
    } else if (key == 'h') {
      RCLCPP_INFO(this->get_logger(),
                  "Controls: +/- Zoom, i/d Inference Size, t/v Draw Lines, q Quit");
    }
  }

  std::vector<SimpleTracker::Track> last_tracks_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MagnesiteConveyorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
