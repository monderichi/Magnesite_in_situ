#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import message_filters
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time
import math
from datetime import datetime
import image_geometry
from collections import deque

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  – registers PointStamped transforms

# ============================================================
# Configuration
# ============================================================
MODEL_PATH          = "/media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/yolo_uv/only_rock.pt"
MAGNESITE_THRESHOLD = 50.0
CONFIDENCE          = 0.35
INFERENCE_SIZE      = 640
MIN_MASK_PIXELS     = 200
BLUR_KERNEL         = (5, 5)
USE_HALF            = False
SKIP_FRAMES         = 1
ZOOM_FACTOR         = 2.0

# ---- Predictive interception trigger -----------------------
# Rocks travel along the belt in the -Y direction of base_link.
# The trigger fires when a rock's base_link Y drops below TRIGGER_Y_BL
# (15 cm in front of the robot centre).  Belt speed is estimated from
# history, and the predicted Y when the robot will be ready is computed.
# Only rocks whose predicted Y stays within the robot workspace are pushed.
TRIGGER_Y_BL    =  0.15   # base_link Y of the detection line (m)
WORKSPACE_Y_MIN = -0.10   # robot workspace near limit in base_link Y (m)
WORKSPACE_Y_MAX =  0.10   # robot workspace far  limit in base_link Y (m)
MIN_BELT_SPEED  =  0.005  # m/s  – ignore rocks slower than 0.5 cm/s
VELOC_HISTORY_LEN = 20    # frames kept per track for velocity estimation

# ---- Robot speed model ------------------------------------
# exec_time = PLANNING_OVERHEAD_S + NOMINAL_MOTION_S / ROBOT_VELOCITY_SCALE
#
# PLANNING_OVERHEAD_S : MoveIt plan + network latency (speed-independent)
# NOMINAL_MOTION_S    : hover + lower motion time at 100% speed (measured)
# ROBOT_VELOCITY_SCALE: must match magnesite_pusher's velocity_scale param
#
# Example at 10% speed:  2.0 + 0.70 / 0.10 = 9.0 s
PLANNING_OVERHEAD_S   = 2.0    # seconds  (tune with 'o'/'O')
NOMINAL_MOTION_S      = 0.70   # seconds at 100% speed (tune with 'n'/'N')
ROBOT_VELOCITY_SCALE  = 0.10   # fraction – match pusher velocity_scale (tune with 'k'/'K')

def center_zoom(frame, zoom):
    h, w = frame.shape[:2]
    crop_w = int(w / zoom)
    crop_h = int(h / zoom)
    x1 = (w - crop_w) // 2
    y1 = (h - crop_h) // 2
    x2 = x1 + crop_w
    y2 = y1 + crop_h
    cropped = frame[y1:y2, x1:x2]
    zoomed  = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
    return zoomed, (x1, y1, x2, y2)

def zoomed_pixel_to_original(zx, zy, zoom, crop_rect, frame_w, frame_h):
    x1, y1, x2, y2 = crop_rect
    crop_w = x2 - x1
    crop_h = y2 - y1
    orig_x = (zx / frame_w) * crop_w + x1
    orig_y = (zy / frame_h) * crop_h + y1
    return int(orig_x), int(orig_y)

def classify_rock_lab_otsu(frame, rock_mask):
    total_pixels = np.count_nonzero(rock_mask)
    if total_pixels < MIN_MASK_PIXELS:
        return False, 0, 0, 0, 0
    ys, xs = np.where(rock_mask > 0)
    y1, y2 = ys.min(), ys.max() + 1
    x1, x2 = xs.min(), xs.max() + 1
    roi_frame = frame[y1:y2, x1:x2]
    roi_mask  = rock_mask[y1:y2, x1:x2]
    lab_roi   = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2LAB)
    l_channel = lab_roi[:, :, 0]
    l_blurred = cv2.GaussianBlur(l_channel, BLUR_KERNEL, 0)
    thresh_val, binary = cv2.threshold(l_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask_bool   = roi_mask > 0
    binary_bool = binary > 0
    white_count = int(np.count_nonzero(mask_bool & binary_bool))
    dark_count  = int(np.count_nonzero(mask_bool & ~binary_bool))
    white_pct = (white_count / total_pixels) * 100
    dark_pct  = (dark_count / total_pixels) * 100
    ratio     = white_count / dark_count if dark_count > 0 else float('inf')
    is_magnesite = white_pct > MAGNESITE_THRESHOLD
    return is_magnesite, white_pct, dark_pct, ratio, thresh_val


class MagnesiteConveyorROS(Node):
    def __init__(self):
        super().__init__('magnesite_conveyor_ros')
        self.bridge = CvBridge()

        self.model = YOLO(MODEL_PATH)
        self.byte_tracker = sv.ByteTrack()

        self.target_pub = self.create_publisher(PointStamped, '/magnesite_target', 10)
        self.alert_pub  = self.create_publisher(String, '/magnesite_alerts', 10)

        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.cam_info_cb, 10)
        self.intrinsics_loaded = False

        self.sub_color = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.zoom           = ZOOM_FACTOR
        self.inference_size = INFERENCE_SIZE

        # ---- Predictive trigger state ----
        self.trigger_y_bl    = TRIGGER_Y_BL
        self.workspace_y_min = WORKSPACE_Y_MIN
        self.workspace_y_max = WORKSPACE_Y_MAX

        # Robot speed model: exec_time = overhead + nominal / velocity_scale
        self.planning_overhead  = PLANNING_OVERHEAD_S
        self.nominal_motion     = NOMINAL_MOTION_S
        self.robot_vel_scale    = ROBOT_VELOCITY_SCALE
        # Derived – recalculated whenever the components change
        self.robot_exec_time    = self._calc_exec_time()
        # Per-track history: {track_id: deque of (time, base_link_y, depth_m)}
        self.track_histories: dict = {}
        # IDs whose push has already been triggered (reset when rock passes verify)
        self.triggered_ids: set = set()

        # TF2 – to convert camera 3-D points to base_link frame
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.skip_frames    = SKIP_FRAMES
        self.frame_count    = 0
        self.last_detections = None
        
        self.drawing_mode = None
        self.current_clicks = []
        
        # We start with default values, but allow drawing
        self.DISPLAY_W = 1280
        self.DISPLAY_H = 800
        
        self.TRIGGER_START = sv.Point(50, 400)
        self.TRIGGER_END   = sv.Point(self.DISPLAY_W - 50, 400)
        self.VERIFY_START  = sv.Point(50, 700)
        self.VERIFY_END    = sv.Point(self.DISPLAY_W - 50, 700)
        
        self.trigger_zone = sv.LineZone(start=self.TRIGGER_START, end=self.TRIGGER_END)
        self.verify_zone = sv.LineZone(start=self.VERIFY_START, end=self.VERIFY_END)
        
        # Annotators
        ROCK_COLOR = sv.Color(r=0,   g=200, b=0)
        MAG_COLOR  = sv.Color(r=0,   g=100, b=255)
        self.palette    = sv.ColorPalette(colors=[ROCK_COLOR, MAG_COLOR])

        self.mask_annotator    = sv.MaskAnnotator(color=self.palette, color_lookup=sv.ColorLookup.CLASS, opacity=0.40)
        self.polygon_annotator = sv.PolygonAnnotator(color=self.palette, color_lookup=sv.ColorLookup.CLASS, thickness=2)
        self.box_annotator     = sv.BoxCornerAnnotator(color=self.palette, color_lookup=sv.ColorLookup.CLASS)
        self.label_annotator   = sv.LabelAnnotator(color=self.palette, color_lookup=sv.ColorLookup.CLASS, text_position=sv.Position.TOP_CENTER)

        self.trigger_annotator = sv.LineZoneAnnotator(color=sv.Color.RED, thickness=2)
        self.verify_annotator  = sv.LineZoneAnnotator(color=sv.Color.YELLOW, thickness=2)
        
        self.fps_monitor = cv2.TickMeter()
        
        cv2.namedWindow('Magnesite Conveyor Pipeline', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Magnesite Conveyor Pipeline', self.DISPLAY_W, self.DISPLAY_H)
        cv2.setMouseCallback('Magnesite Conveyor Pipeline', self.mouse_callback)
        
        self.get_logger().info("CV Node Initialized and waiting for camera topics...")

    def cam_info_cb(self, msg):
        self.cam_model.fromCameraInfo(msg)
        self.intrinsics_loaded = True
        self.destroy_subscription(self.cam_info_sub)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.drawing_mode is not None:
            self.current_clicks.append((x, y))
            if len(self.current_clicks) == 2:
                p1 = sv.Point(self.current_clicks[0][0], self.current_clicks[0][1])
                p2 = sv.Point(self.current_clicks[1][0], self.current_clicks[1][1])
                if self.drawing_mode == 'TRIGGER':
                    self.TRIGGER_START, self.TRIGGER_END = p1, p2
                    self.trigger_zone = sv.LineZone(start=p1, end=p2)
                    print(f"Trigger Line updated")
                elif self.drawing_mode == 'VERIFY':
                    self.VERIFY_START, self.VERIFY_END = p1, p2
                    self.verify_zone = sv.LineZone(start=p1, end=p2)
                    print(f"Verify Line updated")
                self.drawing_mode = None
                self.current_clicks = []

    def get_valid_depth(self, depth_img, x, y, max_radius=5):
        h, w = depth_img.shape
        x, y = int(x), int(y)
        if 0 <= x < w and 0 <= y < h:
            d = depth_img[y, x]
            if d > 0: return float(d)
        for r in range(1, max_radius + 1):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if depth_img[ny, nx] > 0:
                            return float(depth_img[ny, nx])
        return 0.0

    # ----------------------------------------------------------
    # TF2 helper
    # ----------------------------------------------------------
    def _to_base_link(self, x_cam: float, y_cam: float, z_cam: float,
                      stamp) -> tuple:
        """Transform a point from camera_color_optical_frame to base_link.
        Returns (x, y, z) in base_link, or None on failure."""
        pt = PointStamped()
        pt.header.stamp    = stamp
        pt.header.frame_id = 'camera_color_optical_frame'
        pt.point.x = float(x_cam)
        pt.point.y = float(y_cam)
        pt.point.z = float(z_cam)
        try:
            transformed = self.tf_buffer.transform(
                pt, 'base_link',
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (transformed.point.x,
                    transformed.point.y,
                    transformed.point.z)
        except Exception as e:
            self.get_logger().warn(
                f'TF base_link transform failed: {e}', throttle_duration_sec=5.0)
            return None

    # ----------------------------------------------------------
    # Robot exec-time model
    # ----------------------------------------------------------
    def _calc_exec_time(self) -> float:
        """exec_time = planning_overhead + nominal_motion / velocity_scale"""
        if self.robot_vel_scale <= 0:
            return 30.0  # safety cap
        return self.planning_overhead + self.nominal_motion / self.robot_vel_scale

    # ----------------------------------------------------------
    # Velocity / alignment helpers
    # ----------------------------------------------------------
    def _update_track_history(self, track_id: int, y_base: float, depth_m: float):
        """Store base_link Y + depth for belt-speed estimation."""
        if track_id not in self.track_histories:
            self.track_histories[track_id] = deque(maxlen=VELOC_HISTORY_LEN)
        self.track_histories[track_id].append(
            (time.monotonic(), y_base, depth_m))

    def _estimate_belt_speed(self, track_id: int) -> float:
        """Return belt speed in m/s along base_link -Y axis (negative = toward robot).
           Returns 0.0 if not enough history."""
        hist = self.track_histories.get(track_id)
        if not hist or len(hist) < 3:
            return 0.0
        t0, y0, _ = hist[0]
        t1, y1, _ = hist[-1]
        dt = t1 - t0
        if dt < 1e-4:
            return 0.0
        return (y1 - y0) / dt  # negative when moving toward robot

    def sync_callback(self, color_msg, depth_msg):
        if not self.intrinsics_loaded:
            return

        self.fps_monitor.start()
        self.frame_count += 1
        
        full_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        
        # ROS realsense gives lower res (e.g. 480x270 or 848x480). Let's resize it to 1280x800 for consistent UI
        full_frame = cv2.resize(full_frame, (self.DISPLAY_W, self.DISPLAY_H))
        zoomed_frame, crop_rect = center_zoom(full_frame, self.zoom)

        run_inference = (self.frame_count % self.skip_frames == 0) or self.last_detections is None

        if run_inference:
            t0 = time.perf_counter()
            results = self.model(
                zoomed_frame, verbose=False, conf=CONFIDENCE,
                imgsz=self.inference_size, half=USE_HALF)[0]
            self.inference_ms = (time.perf_counter() - t0) * 1000

            detections = sv.Detections.from_ultralytics(results)

            # LAB + OTSU CLASSIFICATION
            classifications = []
            for i in range(len(detections)):
                if (detections.mask is not None
                        and i < len(detections.mask)
                        and detections.mask[i] is not None):
                    rock_mask = (detections.mask[i].astype(np.uint8)) * 255
                else:
                    bx1, by1, bx2, by2 = map(int, detections.xyxy[i])
                    rock_mask = np.zeros(zoomed_frame.shape[:2], dtype=np.uint8)
                    rock_mask[by1:by2, bx1:bx2] = 255

                is_mag, w_pct, d_pct, ratio, thresh = classify_rock_lab_otsu(
                    zoomed_frame, rock_mask)
                classifications.append({'is_mag': is_mag})

            class_ids = np.array(
                [1 if c['is_mag'] else 0 for c in classifications], dtype=int)
            detections.class_id = class_ids

            tracked_detections = self.byte_tracker.update_with_detections(
                detections=detections)
            self.last_detections = tracked_detections
            fresh_inference = True
        else:
            tracked_detections = self.last_detections
            self.inference_ms = 0
            fresh_inference = False

        # ============================================================
        # TRIGGER LOGIC  (predictive interception)
        # ============================================================
        n = len(tracked_detections) if tracked_detections else 0

        if fresh_inference and tracked_detections and n > 0:
            # Keep verify zone updated (pixel-space, for success monitoring)
            crossed_verify_in, crossed_verify_out = self.verify_zone.trigger(
                detections=tracked_detections)
            # Tick trigger zone too (keeps its counters valid, even if unused)
            self.trigger_zone.trigger(detections=tracked_detections)

            df_h, df_w = depth_img.shape

            for i in range(n):
                tracker_id = tracked_detections.tracker_id[i]
                cls_id     = tracked_detections.class_id[i]

                zx, zy, zw, zh = tracked_detections.xyxy[i]
                center_x_disp = (zx + zw) / 2.0
                center_y_disp = (zy + zh) / 2.0

                orig_x, orig_y = zoomed_pixel_to_original(
                    center_x_disp, center_y_disp, self.zoom,
                    crop_rect, self.DISPLAY_W, self.DISPLAY_H)
                native_x = int(orig_x * (df_w / self.DISPLAY_W))
                native_y = int(orig_y * (df_h / self.DISPLAY_H))
                depth_mm = self.get_valid_depth(depth_img, native_x, native_y)

                if depth_mm > 0 and cls_id == 1:
                    depth_m = depth_mm / 1000.0
                    vector  = self.cam_model.projectPixelTo3dRay(
                        (native_x, native_y))
                    xc = float(vector[0] * depth_m)
                    yc = float(vector[1] * depth_m)
                    zc = float(depth_m)

                    bl = self._to_base_link(xc, yc, zc,
                                            self.get_clock().now().to_msg())
                    if bl is not None:
                        bl_x, bl_y, bl_z = bl
                        self._update_track_history(tracker_id, bl_y, depth_m)

                        # ---- PREDICTIVE TRIGGER ----
                        # Fires ONCE when rock's base_link Y crosses below
                        # TRIGGER_Y_BL (15 cm before robot centre).
                        if tracker_id not in self.triggered_ids:
                            hist = self.track_histories.get(tracker_id)
                            if hist and len(hist) >= 2:
                                prev_y = hist[-2][1]
                                curr_y = bl_y
                                # Detect crossing: was above line, now at/below
                                if prev_y > self.trigger_y_bl >= curr_y:
                                    speed_y = self._estimate_belt_speed(tracker_id)
                                    exec_t  = self._calc_exec_time()
                                    if abs(speed_y) < MIN_BELT_SPEED:
                                        self.get_logger().warn(
                                            f"[TRIGGER] ID {tracker_id}: belt speed "
                                            f"{speed_y*100:.1f} cm/s too slow, skip.")
                                    else:
                                        # Predict Y when robot will be ready
                                        y_pred = curr_y + speed_y * exec_t
                                        self.get_logger().info(
                                            f"[TRIGGER] ID {tracker_id}: "
                                            f"Y={curr_y:.3f}m, "
                                            f"speed={speed_y*100:.1f} cm/s, "
                                            f"exec_t={exec_t:.1f}s "
                                            f"(overhead={self.planning_overhead:.1f}s + "
                                            f"motion={self.nominal_motion:.2f}s / "
                                            f"vel={self.robot_vel_scale:.0%}), "
                                            f"Y_predicted={y_pred:.3f}m")
                                        if self.workspace_y_min <= y_pred <= self.workspace_y_max:
                                            self.publish_target_base_link(
                                                bl_x, y_pred, bl_z, tracker_id)
                                            self.triggered_ids.add(tracker_id)
                                        else:
                                            self.get_logger().warn(
                                                f"[TRIGGER] ID {tracker_id}: predicted "
                                                f"Y={y_pred:.3f}m outside workspace "
                                                f"[{self.workspace_y_min:.2f}, "
                                                f"{self.workspace_y_max:.2f}]. Skip.")

                # ---- VERIFY line (success monitoring, pixel-space) ----
                if crossed_verify_in[i] or crossed_verify_out[i]:
                    if cls_id == 1:
                        alert_msg = (f"ALERT! Magnesite ID {tracker_id} "
                                     f"reached Verification Line!")
                        self.get_logger().warning(alert_msg)
                        self.alert_pub.publish(String(data=alert_msg))
                        # Clear state so a re-appearing rock can be pushed again
                        self.triggered_ids.discard(tracker_id)
                        self.track_histories.pop(tracker_id, None)

        # ============================================================
        # VISUALIZATION
        # ============================================================
        annotated = zoomed_frame.copy()

        # ---- Draw predictive trigger info on OSD ----
        exec_t = self._calc_exec_time()
        cv2.putText(annotated,
                    f"Trigger Y={self.trigger_y_bl:+.2f}m | "
                    f"Workspace Y=[{self.workspace_y_min:.2f},{self.workspace_y_max:.2f}]m",
                    (10, self.DISPLAY_H - 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 80), 1)
        cv2.putText(annotated,
                    f"ExecT={exec_t:.1f}s = overhead({self.planning_overhead:.1f}s) + "
                    f"motion({self.nominal_motion:.2f}s) / vel({self.robot_vel_scale:.0%})",
                    (10, self.DISPLAY_H - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)

        if n > 0:
            if tracked_detections.mask is not None:
                annotated = self.mask_annotator.annotate(
                    scene=annotated, detections=tracked_detections)
                annotated = self.polygon_annotator.annotate(
                    scene=annotated, detections=tracked_detections)
            annotated = self.box_annotator.annotate(
                scene=annotated, detections=tracked_detections)
            labels = []
            for tid, cid in zip(tracked_detections.tracker_id,
                                 tracked_detections.class_id):
                speed = self._estimate_belt_speed(tid)
                spd_str = f" {speed*100:.0f}cm/s" if abs(speed) > 0.001 else ""
                labels.append(
                    f"ID:{tid} {'MAG' if cid==1 else 'ROCK'}{spd_str}")
            annotated = self.label_annotator.annotate(
                scene=annotated, detections=tracked_detections, labels=labels)

        annotated = self.trigger_annotator.annotate(
            annotated, line_counter=self.trigger_zone)
        annotated = self.verify_annotator.annotate(
            annotated, line_counter=self.verify_zone)

        cv2.putText(annotated, "TRIGGER LINE (fallback)",
                    (int(self.TRIGGER_START.x), int(self.TRIGGER_START.y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(annotated, "VERIFICATION LINE",
                    (int(self.VERIFY_START.x), int(self.VERIFY_START.y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        if self.drawing_mode:
            cv2.putText(annotated, f"CLICK 2 POINTS FOR {self.drawing_mode} LINE", (self.DISPLAY_W//2 - 300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            for c in self.current_clicks:
                cv2.circle(annotated, c, 5, (255,255,255), -1)

        cv2.putText(annotated, f"ZOOM {self.zoom:.1f}x", (self.DISPLAY_W // 2 - 50, self.DISPLAY_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

        self.fps_monitor.stop()
        
        # Info panel
        panel_h, panel_w = 130, 520
        roi_p = annotated[5:panel_h, 5:panel_w]
        annotated[5:panel_h, 5:panel_w] = (roi_p * 0.4).astype(np.uint8)

        cv2.putText(annotated, f"FPS: {self.fps_monitor.getFPS():.1f} | Inference: {getattr(self, 'inference_ms', 0):.0f}ms | Res: {self.inference_size}", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(annotated, "Press 'h' for controls | 'q' to quit", (15, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (150, 150, 150), 1)
        cv2.putText(annotated, "Press 't' draw trigger | 'v' draw verify", (15, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        cv2.imshow('Magnesite Conveyor Pipeline', annotated)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:
            rclpy.shutdown()
        elif key == ord('t'):
            self.drawing_mode = 'TRIGGER'
            self.current_clicks = []
        elif key == ord('v'):
            self.drawing_mode = 'VERIFY'
            self.current_clicks = []
        elif key == ord('+') or key == ord('='):
            self.zoom = min(self.zoom + 0.25, 8.0)
        elif key == ord('-') or key == ord('_'):
            self.zoom = max(self.zoom - 0.25, 1.0)
        elif key in [ord('i'), ord('I')]:
            self.inference_size = min(self.inference_size + 64, 1280)
            self.get_logger().info(f"Resolution increased to {self.inference_size}")
        elif key in [ord('d'), ord('D')]:
            self.inference_size = max(self.inference_size - 64, 320)
            self.get_logger().info(f"Resolution decreased to {self.inference_size}")
        # Tuning keys
        elif key == ord('p'):
            self.trigger_y_bl = round(self.trigger_y_bl + 0.01, 3)
            self.get_logger().info(f"Trigger Y = {self.trigger_y_bl:+.3f} m")
        elif key == ord(';'):
            self.trigger_y_bl = round(self.trigger_y_bl - 0.01, 3)
            self.get_logger().info(f"Trigger Y = {self.trigger_y_bl:+.3f} m")
        # Planning overhead (speed-independent part of exec time)
        elif key == ord('o'):
            self.planning_overhead = round(self.planning_overhead + 0.5, 1)
            self.get_logger().info(
                f"Planning overhead = {self.planning_overhead:.1f}s  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        elif key == ord('O'):
            self.planning_overhead = max(0.0, round(self.planning_overhead - 0.5, 1))
            self.get_logger().info(
                f"Planning overhead = {self.planning_overhead:.1f}s  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        # Nominal motion time at 100% speed
        elif key == ord('n'):
            self.nominal_motion = round(self.nominal_motion + 0.05, 2)
            self.get_logger().info(
                f"Nominal motion = {self.nominal_motion:.2f}s  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        elif key == ord('N'):
            self.nominal_motion = max(0.0, round(self.nominal_motion - 0.05, 2))
            self.get_logger().info(
                f"Nominal motion = {self.nominal_motion:.2f}s  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        # Robot velocity scale (must match pusher's velocity_scale param)
        elif key == ord('k'):
            self.robot_vel_scale = min(1.0, round(self.robot_vel_scale + 0.01, 2))
            self.get_logger().info(
                f"Robot vel scale = {self.robot_vel_scale:.0%}  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        elif key == ord('K'):
            self.robot_vel_scale = max(0.01, round(self.robot_vel_scale - 0.01, 2))
            self.get_logger().info(
                f"Robot vel scale = {self.robot_vel_scale:.0%}  "
                f"→ exec_t = {self._calc_exec_time():.1f}s")
        elif key == ord('h'):
            print(
                "Controls:\n"
                "  +/-  Zoom | i/d  Resolution | t/v  Draw lines\n"
                "  p/;  Trigger Y (+/-1cm)\n"
                "  o/O  Planning overhead (+/-0.5s)\n"
                "  n/N  Nominal motion time at 100% (+/-0.05s)\n"
                "  k/K  Robot velocity scale (+/-1%)\n"
                "  q    Quit")

    def publish_target(self, x, y, z, tracker_id):
        """Publish target in camera_color_optical_frame (legacy)."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Published target ID {tracker_id} [cam]: "
            f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    def publish_target_base_link(self, x, y, z, tracker_id):
        """Publish predicted target in base_link frame directly."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Published predicted target ID {tracker_id} [base_link]: "
            f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = MagnesiteConveyorROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
