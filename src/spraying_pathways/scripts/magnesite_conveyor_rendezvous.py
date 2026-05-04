#!/usr/bin/env python3
"""
magnesite_conveyor_rendezvous.py

Integrated conveyor vision node with:
  - YOLOv8-seg + ByteTrack detection & tracking
  - LAB + Otsu classification (magnesite vs regular rock)
  - Kalman Filter per track for Y position / velocity
  - 3-D edge detection: finds the object edge closer to base_link in X
  - Analytical rendezvous solver: computes optimal Y intercept point
  - Push timing: robot arrives early, preps, and pushes when object reaches EE

Publishes PointStamped targets on /magnesite_target with:
  - X = object edge closer to base_link (push contact point)
  - Y = computed rendezvous point (object at this Y when robot ready to push)
  - Z = measured from depth (constant for flat belt)

Usage:
    ros2 run spraying_pathways magnesite_conveyor_rendezvous.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float64
import message_filters
from cv_bridge import CvBridge

import cv2
import image_geometry
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time
from typing import Optional, Tuple
from collections import deque

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from filterpy.kalman import KalmanFilter

# Import local rendezvous solver
from conveyor_rendezvous_solver import ConveyorRendezvousSolver, RendezvousResult

# ============================================================
# Configuration
# ============================================================
MODEL_PATH = (
    "/media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/"
    "yolo_uv/only_rock.pt"
)
MAGNESITE_THRESHOLD = 50.0
CONFIDENCE = 0.35
INFERENCE_SIZE = 640
MIN_MASK_PIXELS = 200
BLUR_KERNEL = (5, 5)
USE_HALF = False
SKIP_FRAMES = 1
ZOOM_FACTOR = 2.0

# ---- Fixed EE Y (park / monitoring pose) -------------------
Y_PARK = 0.255

# ---- Robot dynamics for rendezvous solver ------------------
# Measure these empirically at your operating velocity_scale:
V_ROBOT_CARTESIAN = 0.035      # m/s  (EE linear speed in Y)
PLANNING_OVERHEAD_S = 2.2      # s    (goPushStart time at 40% scale, measured)
NOMINAL_MOTION_S = 0.70        # s    (hover time at 100% speed)
ROBOT_VELOCITY_SCALE = 0.40    # must match pusher node (was 0.10)

# Time from robot arrival at hover to START of push (orient + lower)
T_PREP_S = 0.7                 # Cartesian push takes ~1.4s, half = 0.7s mid-point

# Workspace limits in base_link Y
WORKSPACE_Y_MIN = -0.10
WORKSPACE_Y_MAX = 0.35

# Tool / pusher width in Y direction (m)
TOOL_WIDTH_Y = 0.06

# ---- Kalman Filter -----------------------------------------
KF_MEASUREMENT_NOISE_R = 0.008
KF_PROCESS_NOISE_POS_Q = 0.001
KF_PROCESS_NOISE_VEL_Q = 0.05

# ---- X/Z EMA smoothing -------------------------------------
XZ_EMA_ALPHA = 0.3

# ---- Belt height -------------------------------------------
# Measure the conveyor belt surface Z in base_link (m).
# The tool tip will be placed PUSH_Z_OFFSET above this surface.
BELT_Z = 0.050                 # adjust to your measured belt height
PUSH_Z_OFFSET = 0.001          # 1 mm above belt surface

# ---- Safety clamps -----------------------------------------
MIN_Z_SAFETY = BELT_Z + PUSH_Z_OFFSET  # 1 mm above belt
RETRIGGER_COOLDOWN_S = 5.0     # same ID cannot re-trigger within 5s

# ---- Trigger tolerances ------------------------------------
MIN_TRACK_AGE = 3
MIN_BELT_SPEED = 0.005

# ============================================================
# Helpers
# ============================================================
def center_zoom(frame, zoom):
    h, w = frame.shape[:2]
    crop_w = int(w / zoom)
    crop_h = int(h / zoom)
    x1 = (w - crop_w) // 2
    y1 = (h - crop_h) // 2
    x2 = x1 + crop_w
    y2 = y1 + crop_h
    cropped = frame[y1:y2, x1:x2]
    zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
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
    roi_mask = rock_mask[y1:y2, x1:x2]
    lab_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2LAB)
    l_channel = lab_roi[:, :, 0]
    l_blurred = cv2.GaussianBlur(l_channel, BLUR_KERNEL, 0)
    thresh_val, binary = cv2.threshold(
        l_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    mask_bool = roi_mask > 0
    binary_bool = binary > 0
    white_count = int(np.count_nonzero(mask_bool & binary_bool))
    dark_count = int(np.count_nonzero(mask_bool & ~binary_bool))
    white_pct = (white_count / total_pixels) * 100
    dark_pct = (dark_count / total_pixels) * 100
    ratio = white_count / dark_count if dark_count > 0 else float('inf')
    is_magnesite = white_pct > MAGNESITE_THRESHOLD
    return is_magnesite, white_pct, dark_pct, ratio, thresh_val


# ============================================================
# Node
# ============================================================
class MagnesiteConveyorRendezvous(Node):
    def __init__(self):
        super().__init__('magnesite_conveyor_rendezvous')
        self.bridge = CvBridge()

        self.model = YOLO(MODEL_PATH)
        self.byte_tracker = sv.ByteTrack()

        self.target_pub = self.create_publisher(
            PointStamped, '/magnesite_target', 10)
        self.alert_pub = self.create_publisher(
            String, '/magnesite_alerts', 10)

        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.cam_info_cb, 10)
        self.intrinsics_loaded = False

        self.sub_color = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.zoom = ZOOM_FACTOR
        self.inference_size = INFERENCE_SIZE

        # ---- Kalman Filter state ---------------------------------
        self.y_park = Y_PARK
        self.track_kfs: dict = {}
        self.track_last_t: dict = {}
        self.track_age: dict = {}
        self.track_xz_smooth: dict = {}
        self.track_prev_y: dict = {}       # {track_id: prev_y_kf} for line crossing
        self.triggered_ids: set = set()
        self.last_trigger_time: dict = {}   # {track_id: monotonic_time}
        self.armed_ids: set = set()         # rocks that have crossed the trigger line

        # ---- Rendezvous solver -----------------------------------
        self.solver = ConveyorRendezvousSolver(
            y_park=Y_PARK,
            v_robot=V_ROBOT_CARTESIAN,
            t_overhead=PLANNING_OVERHEAD_S,
            y_workspace_min=WORKSPACE_Y_MIN,
            y_workspace_max=WORKSPACE_Y_MAX,
            t_push=T_PREP_S,
            tool_width_y=TOOL_WIDTH_Y,
        )

        # Online exec-time measurement from pusher feedback
        self.measured_exec_t = None
        self.create_subscription(
            Float64, '/magnesite_exec_time',
            self.exec_time_cb, 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.skip_frames = SKIP_FRAMES
        self.frame_count = 0
        self.last_detections = None

        self.drawing_mode = None
        self.current_clicks = []

        # Display
        self.DISPLAY_W = 1280
        self.DISPLAY_H = 800

        self.TRIGGER_START = sv.Point(50, 400)
        self.TRIGGER_END = sv.Point(self.DISPLAY_W - 50, 400)
        self.VERIFY_START = sv.Point(50, 700)
        self.VERIFY_END = sv.Point(self.DISPLAY_W - 50, 700)

        self.trigger_zone = sv.LineZone(
            start=self.TRIGGER_START, end=self.TRIGGER_END)
        self.verify_zone = sv.LineZone(
            start=self.VERIFY_START, end=self.VERIFY_END)

        # Annotators
        ROCK_COLOR = sv.Color(r=0, g=200, b=0)
        MAG_COLOR = sv.Color(r=0, g=100, b=255)
        self.palette = sv.ColorPalette(colors=[ROCK_COLOR, MAG_COLOR])

        self.mask_annotator = sv.MaskAnnotator(
            color=self.palette, color_lookup=sv.ColorLookup.CLASS, opacity=0.40)
        self.polygon_annotator = sv.PolygonAnnotator(
            color=self.palette, color_lookup=sv.ColorLookup.CLASS, thickness=2)
        self.box_annotator = sv.BoxCornerAnnotator(
            color=self.palette, color_lookup=sv.ColorLookup.CLASS)
        self.label_annotator = sv.LabelAnnotator(
            color=self.palette, color_lookup=sv.ColorLookup.CLASS,
            text_position=sv.Position.TOP_CENTER)

        self.trigger_annotator = sv.LineZoneAnnotator(
            color=sv.Color.RED, thickness=2)
        self.verify_annotator = sv.LineZoneAnnotator(
            color=sv.Color.YELLOW, thickness=2)

        self.fps_monitor = cv2.TickMeter()

        cv2.namedWindow('Magnesite Conveyor Rendezvous', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Magnesite Conveyor Rendezvous',
                         self.DISPLAY_W, self.DISPLAY_H)
        cv2.setMouseCallback('Magnesite Conveyor Rendezvous',
                             self.mouse_callback)

        self.get_logger().info(
            "Rendezvous node init. Y_PARK=%.3f | v_robot=%.3f m/s | "
            "t_prep=%.1fs | tool_w=%.3fm" % (
                self.y_park, V_ROBOT_CARTESIAN, T_PREP_S, TOOL_WIDTH_Y))

    # ------------------------------------------------------------------
    # Camera & UI callbacks
    # ------------------------------------------------------------------
    def cam_info_cb(self, msg):
        self.cam_model.fromCameraInfo(msg)
        self.intrinsics_loaded = True
        self.destroy_subscription(self.cam_info_sub)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.drawing_mode is not None:
            self.current_clicks.append((x, y))
            if len(self.current_clicks) == 2:
                p1 = sv.Point(self.current_clicks[0][0],
                              self.current_clicks[0][1])
                p2 = sv.Point(self.current_clicks[1][0],
                              self.current_clicks[1][1])
                if self.drawing_mode == 'TRIGGER':
                    self.TRIGGER_START, self.TRIGGER_END = p1, p2
                    self.trigger_zone = sv.LineZone(start=p1, end=p2)
                    print("Trigger Line updated")
                elif self.drawing_mode == 'VERIFY':
                    self.VERIFY_START, self.VERIFY_END = p1, p2
                    self.verify_zone = sv.LineZone(start=p1, end=p2)
                    print("Verify Line updated")
                self.drawing_mode = None
                self.current_clicks = []

    def get_valid_depth(self, depth_img, x, y, max_radius=5):
        h, w = depth_img.shape
        x, y = int(x), int(y)
        if 0 <= x < w and 0 <= y < h:
            d = depth_img[y, x]
            if d > 0:
                return float(d)
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if depth_img[ny, nx] > 0:
                            return float(depth_img[ny, nx])
        return 0.0

    # ------------------------------------------------------------------
    # TF2 helper
    # ------------------------------------------------------------------
    def _to_base_link(self, x_cam, y_cam, z_cam, stamp) -> Optional[Tuple]:
        pt = PointStamped()
        pt.header.stamp = stamp
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
                f'TF base_link transform failed: {e}',
                throttle_duration_sec=5.0)
            return None

    # ------------------------------------------------------------------
    # Exec-time feedback from pusher
    # ------------------------------------------------------------------
    def exec_time_cb(self, msg: Float64):
        self.measured_exec_t = float(msg.data)
        self.get_logger().info(
            f"Online exec_time measured: {self.measured_exec_t:.2f}s")

    # ------------------------------------------------------------------
    # Kalman Filter helpers
    # ------------------------------------------------------------------
    def _init_kf(self, y_meas: float) -> KalmanFilter:
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.array([[y_meas], [0.0]])
        kf.F = np.array([[1.0, 1.0], [0.0, 1.0]])
        kf.H = np.array([[1.0, 0.0]])
        kf.P = np.array([[0.05, 0.0], [0.0, 0.5]])
        kf.R = np.array([[KF_MEASUREMENT_NOISE_R]])
        kf.Q = np.array([
            [KF_PROCESS_NOISE_POS_Q, 0.0],
            [0.0, KF_PROCESS_NOISE_VEL_Q]
        ])
        return kf

    def _update_kf(self, track_id: int, t: float, y_meas: float):
        if track_id not in self.track_kfs:
            self.track_kfs[track_id] = self._init_kf(y_meas)
            self.track_last_t[track_id] = t
            self.track_age[track_id] = 1
            return y_meas, 0.0

        kf = self.track_kfs[track_id]
        dt = t - self.track_last_t[track_id]
        if dt > 1e-4:
            kf.F[0, 1] = dt
            kf.predict()
            kf.update(np.array([[y_meas]]))
            self.track_last_t[track_id] = t
            self.track_age[track_id] += 1
        return float(kf.x[0, 0]), float(kf.x[1, 0])

    def _smooth_xz(self, track_id: int, x_meas: float, z_meas: float):
        if track_id not in self.track_xz_smooth:
            self.track_xz_smooth[track_id] = {'x': x_meas, 'z': z_meas}
            return x_meas, z_meas
        s = self.track_xz_smooth[track_id]
        s['x'] = XZ_EMA_ALPHA * x_meas + (1.0 - XZ_EMA_ALPHA) * s['x']
        s['z'] = XZ_EMA_ALPHA * z_meas + (1.0 - XZ_EMA_ALPHA) * s['z']
        return s['x'], s['z']

    # ------------------------------------------------------------------
    # Edge detection: find object edge closer to base_link in X
    # ------------------------------------------------------------------
    def _get_edge_x_base_link(
        self,
        detection_idx: int,
        tracked_detections,
        zoomed_frame,
        crop_rect,
        depth_img,
        stamp,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Compute the object edge closer to base_link (minimum X) and the
        center Z. Returns (edge_x, center_y, center_z) in base_link, or None.
        """
        # --- 1. Get mask or bounding box in zoomed coordinates ----------
        has_mask = (
            tracked_detections.mask is not None
            and detection_idx < len(tracked_detections.mask)
            and tracked_detections.mask[detection_idx] is not None
        )

        df_h, df_w = depth_img.shape

        # Helper to project a single pixel to base_link
        def pixel_to_bl(px, py, d_mm):
            if d_mm <= 0:
                return None
            d_m = d_mm / 1000.0
            ray = self.cam_model.projectPixelTo3dRay((px, py))
            xc = float(ray[0] * d_m)
            yc = float(ray[1] * d_m)
            zc = float(d_m)
            return self._to_base_link(xc, yc, zc, stamp)

        # --- 2. Mask-based edge detection (preferred) ------------------
        if has_mask:
            mask = tracked_detections.mask[detection_idx]
            mask_ys, mask_xs = np.where(mask > 0)
            if len(mask_xs) == 0:
                has_mask = False
            else:
                # Leftmost and rightmost pixels in image X
                left_idx = int(mask_xs.argmin())
                right_idx = int(mask_xs.argmax())

                left_zx = int(mask_xs[left_idx])
                left_zy = int(mask_ys[left_idx])
                right_zx = int(mask_xs[right_idx])
                right_zy = int(mask_ys[right_idx])

                # Convert zoomed mask pixels → original → depth native
                def mask_pix_to_native(zx, zy):
                    ox, oy = zoomed_pixel_to_original(
                        zx, zy, self.zoom, crop_rect,
                        self.DISPLAY_W, self.DISPLAY_H)
                    nx = int(ox * (df_w / self.DISPLAY_W))
                    ny = int(oy * (df_h / self.DISPLAY_H))
                    return nx, ny

                nl_x, nl_y = mask_pix_to_native(left_zx, left_zy)
                nr_x, nr_y = mask_pix_to_native(right_zx, right_zy)

                dl = self.get_valid_depth(depth_img, nl_x, nl_y)
                dr = self.get_valid_depth(depth_img, nr_x, nr_y)

                bl_left = pixel_to_bl(nl_x, nl_y, dl)
                bl_right = pixel_to_bl(nr_x, nr_y, dr)

                if bl_left is None or bl_right is None:
                    has_mask = False
                else:
                    # Edge closer to base_link = smaller base_link X
                    edge_x = min(bl_left[0], bl_right[0])
                    # Center Y and Z from mask centroid depth
                    cy_z = int(np.mean(mask_ys))
                    cx_z = int(np.mean(mask_xs))
                    ox_c, oy_c = zoomed_pixel_to_original(
                        cx_z, cy_z, self.zoom, crop_rect,
                        self.DISPLAY_W, self.DISPLAY_H)
                    nc_x = int(ox_c * (df_w / self.DISPLAY_W))
                    nc_y = int(oy_c * (df_h / self.DISPLAY_H))
                    dc = self.get_valid_depth(depth_img, nc_x, nc_y)
                    bl_center = pixel_to_bl(nc_x, nc_y, dc)
                    if bl_center is None:
                        return None
                    return (edge_x, bl_center[1], bl_center[2])

        # --- 3. Bounding-box fallback ----------------------------------
        if not has_mask:
            zx1, zy1, zx2, zy2 = tracked_detections.xyxy[detection_idx]
            zcy = (zy1 + zy2) / 2.0

            # Left and right edges at mid-height
            def bb_pix_to_native(zx, zy):
                ox, oy = zoomed_pixel_to_original(
                    zx, zy, self.zoom, crop_rect,
                    self.DISPLAY_W, self.DISPLAY_H)
                nx = int(ox * (df_w / self.DISPLAY_W))
                ny = int(oy * (df_h / self.DISPLAY_H))
                return nx, ny

            nl_x, nl_y = bb_pix_to_native(zx1, zcy)
            nr_x, nr_y = bb_pix_to_native(zx2, zcy)
            nc_x, nc_y = bb_pix_to_native((zx1 + zx2) / 2.0, zcy)

            dl = self.get_valid_depth(depth_img, nl_x, nl_y)
            dr = self.get_valid_depth(depth_img, nr_x, nr_y)
            dc = self.get_valid_depth(depth_img, nc_x, nc_y)

            bl_left = pixel_to_bl(nl_x, nl_y, dl)
            bl_right = pixel_to_bl(nr_x, nr_y, dr)
            bl_center = pixel_to_bl(nc_x, nc_y, dc)

            if bl_left is None or bl_right is None or bl_center is None:
                return None

            edge_x = min(bl_left[0], bl_right[0])
            return (edge_x, bl_center[1], bl_center[2])

        return None

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def sync_callback(self, color_msg, depth_msg):
        if not self.intrinsics_loaded:
            return

        self.fps_monitor.start()
        self.frame_count += 1

        full_frame = self.bridge.imgmsg_to_cv2(
            color_msg, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough')

        full_frame = cv2.resize(
            full_frame, (self.DISPLAY_W, self.DISPLAY_H))
        zoomed_frame, crop_rect = center_zoom(full_frame, self.zoom)

        run_inference = (
            (self.frame_count % self.skip_frames == 0)
            or self.last_detections is None
        )

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
        # KF UPDATE + RENDEZVOUS + EDGE DETECTION
        # ============================================================
        n = len(tracked_detections) if tracked_detections else 0
        active_ids = set()
        t_now = time.monotonic()

        if fresh_inference and tracked_detections and n > 0:
            crossed_verify_in, crossed_verify_out = self.verify_zone.trigger(
                detections=tracked_detections)
            # Capture trigger line crossings to arm rocks for firing
            crossed_trigger_in, crossed_trigger_out = self.trigger_zone.trigger(
                detections=tracked_detections)

            for i in range(n):
                tracker_id = tracked_detections.tracker_id[i]
                cls_id = tracked_detections.class_id[i]
                active_ids.add(tracker_id)

                # ---- Stage 1: Arm rock when it crosses the trigger line ----
                # Accept either crossing direction (in OR out) so it works
                # regardless of which way the user drew the trigger line.
                if (crossed_trigger_in[i] or crossed_trigger_out[i]) and cls_id == 1:
                    if tracker_id not in self.armed_ids:
                        self.armed_ids.add(tracker_id)
                        self.get_logger().info(
                            f"[ARMED] ID {tracker_id}: crossed trigger line, armed for push")

                if cls_id != 1:
                    continue  # only process magnesite

                # ---- 3-D edge detection --------------------------------
                edge_result = self._get_edge_x_base_link(
                    i, tracked_detections, zoomed_frame,
                    crop_rect, depth_img,
                    self.get_clock().now().to_msg())

                if edge_result is None:
                    continue

                edge_x, bl_y, bl_z = edge_result

                # Smooth X/Z independently (Y is handled by KF)
                x_smooth, z_smooth = self._smooth_xz(
                    tracker_id, edge_x, bl_z)

                # ---- Kalman Filter for Y -----------------------------
                y_kf, vy_kf = self._update_kf(tracker_id, t_now, bl_y)

                # ---- Algorithm runs in background (monitoring only) ----
                age = self.track_age.get(tracker_id, 0)
                res = self.solver.solve(
                    y_obj=y_kf,
                    v_obj=vy_kf,
                    push_compensation="end",
                )
                if age >= MIN_TRACK_AGE and abs(vy_kf) > MIN_BELT_SPEED:
                    self.get_logger().info(
                        f"[RENDEZVOUS] ID {tracker_id}: "
                        f"y_kf={y_kf:.3f} vy={vy_kf*100:.1f}cm/s "
                        f"edge_x={edge_x:.3f} "
                        f"y_r={res.y_r:.3f} t_r={res.t_r:.1f}s "
                        f"feasible={res.feasible}",
                        throttle_duration_sec=0.5)

                # Fixed push Z: 1 mm above belt surface
                z_push = BELT_Z + PUSH_Z_OFFSET

                # ---- Trigger: fire when belt travel time = robot travel time ----
                # Our robot goes to a FIXED push position (not a moving rendezvous).
                # Correct condition: fire when the rock's remaining travel time to
                # Y_PARK equals the robot's time to reach push-start position.
                #
                # t_goPushStart ≈ 41% of full cycle (measured: 2.16s / 5.29s).
                # Fire when: (y_kf - Y_PARK) / |vy| <= t_goPushStart
                # i.e.:       y_kf <= Y_PARK + |vy| * t_goPushStart
                GOTO_PUSH_FRACTION = 0.41   # goPushStart / full_cycle
                GOTO_PUSH_FALLBACK_S = 2.2  # first-run estimate
                if self.measured_exec_t is not None:
                    t_goPushStart = self.measured_exec_t * GOTO_PUSH_FRACTION
                else:
                    t_goPushStart = GOTO_PUSH_FALLBACK_S

                trigger_y = self.y_park + abs(vy_kf) * t_goPushStart
                should_fire = (y_kf <= trigger_y)

                # Cooldown check
                last_t = self.last_trigger_time.get(tracker_id, 0.0)
                cooldown_ok = (t_now - last_t) > RETRIGGER_COOLDOWN_S

                # ---- Stage 2: Fire push (only if ARMED by trigger line) ----
                if (should_fire
                        and tracker_id in self.armed_ids
                        and tracker_id not in self.triggered_ids
                        and age >= MIN_TRACK_AGE
                        and vy_kf < -MIN_BELT_SPEED
                        and cooldown_ok):
                    y_at_push = y_kf + vy_kf * t_goPushStart
                    if WORKSPACE_Y_MIN <= y_at_push <= WORKSPACE_Y_MAX:
                        self.publish_target_base_link(
                            x_smooth, self.y_park, z_push, tracker_id)
                        self.triggered_ids.add(tracker_id)
                        self.last_trigger_time[tracker_id] = t_now
                        self.get_logger().info(
                            f"[TRIGGER] ID {tracker_id}: FIRE "
                            f"y_rock={y_kf:.3f} trigger_y={trigger_y:.3f} "
                            f"y_at_push={y_at_push:.3f} "
                            f"t_goto={t_goPushStart:.2f}s vy={vy_kf*100:.1f}cm/s "
                            f"[solver: y_r={res.y_r:.3f} t_r={res.t_r:.1f}s]")
                    else:
                        self.get_logger().warn(
                            f"[TRIGGER] ID {tracker_id}: y_at_push={y_at_push:.3f} "
                            f"outside workspace [{WORKSPACE_Y_MIN:.2f},{WORKSPACE_Y_MAX:.2f}], skip.")

                # ---- Verify line: disarm + alert -------------------------
                if crossed_verify_in[i] or crossed_verify_out[i]:
                    alert_msg = (
                        f"ALERT! Magnesite ID {tracker_id} "
                        f"reached Verification Line!")
                    self.get_logger().warning(alert_msg)
                    self.alert_pub.publish(String(data=alert_msg))
                    self.triggered_ids.discard(tracker_id)
                    self.armed_ids.discard(tracker_id)   # disarm so it can re-arm next pass
                    for d in (self.track_kfs, self.track_last_t,
                              self.track_age, self.track_xz_smooth):
                        d.pop(tracker_id, None)

        # Purge stale tracks
        stale = [tid for tid in self.track_kfs if tid not in active_ids]
        for tid in stale:
            self.track_kfs.pop(tid, None)
            self.track_last_t.pop(tid, None)
            self.track_age.pop(tid, None)
            self.track_xz_smooth.pop(tid, None)
            self.track_prev_y.pop(tid, None)
            self.triggered_ids.discard(tid)
            self.armed_ids.discard(tid)
            self.last_trigger_time.pop(tid, None)

        # ============================================================
        # VISUALIZATION
        # ============================================================
        annotated = zoomed_frame.copy()

        exec_t = self.solver._calc_exec_time()
        cv2.putText(
            annotated,
            f"Y_PARK={self.y_park:+.3f}m | ExecT={exec_t:.1f}s | "
            f"v_robot={V_ROBOT_CARTESIAN:.3f}m/s",
            (10, self.DISPLAY_H - 55),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
        cv2.putText(
            annotated,
            f"t_prep={T_PREP_S:.1f}s | tool_w={TOOL_WIDTH_Y:.3f}m | "
            f"comp=end",
            (10, self.DISPLAY_H - 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 80), 1)

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
                vy = 0.0
                y_r_str = ""
                if tid in self.track_kfs:
                    vy = float(self.track_kfs[tid].x[1, 0])
                    if cid == 1 and vy < -0.001:
                        y_kf = float(self.track_kfs[tid].x[0, 0])
                        # Show predicted rendezvous Y if we computed it
                        # (simplified display)
                        y_r_str = f" y_r={y_kf:.2f}"
                spd_str = f" {vy*100:.0f}cm/s" if abs(vy) > 0.001 else ""
                labels.append(
                    f"ID:{tid} {'MAG' if cid == 1 else 'ROCK'}{spd_str}{y_r_str}")
            annotated = self.label_annotator.annotate(
                scene=annotated, detections=tracked_detections, labels=labels)

        annotated = self.trigger_annotator.annotate(
            annotated, line_counter=self.trigger_zone)
        annotated = self.verify_annotator.annotate(
            annotated, line_counter=self.verify_zone)

        cv2.putText(
            annotated, "TRIGGER LINE (visual)",
            (int(self.TRIGGER_START.x), int(self.TRIGGER_START.y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(
            annotated, "VERIFICATION LINE",
            (int(self.VERIFY_START.x), int(self.VERIFY_START.y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.drawing_mode:
            cv2.putText(
                annotated,
                f"CLICK 2 POINTS FOR {self.drawing_mode} LINE",
                (self.DISPLAY_W // 2 - 300, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            for c in self.current_clicks:
                cv2.circle(annotated, c, 5, (255, 255, 255), -1)

        cv2.putText(
            annotated, f"ZOOM {self.zoom:.1f}x",
            (self.DISPLAY_W // 2 - 50, self.DISPLAY_H - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

        self.fps_monitor.stop()

        # Info panel
        panel_h, panel_w = 130, 520
        roi_p = annotated[5:panel_h, 5:panel_w]
        annotated[5:panel_h, 5:panel_w] = (roi_p * 0.4).astype(np.uint8)

        cv2.putText(
            annotated,
            f"FPS: {self.fps_monitor.getFPS():.1f} | "
            f"Inference: {getattr(self, 'inference_ms', 0):.0f}ms | "
            f"Res: {self.inference_size}",
            (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(
            annotated,
            "Press 'h' for controls | 'q' to quit",
            (15, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (150, 150, 150), 1)
        cv2.putText(
            annotated,
            "Press 'r' draw trigger | 'v' draw verify | 'y/g' tune Y_PARK",
            (15, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        cv2.imshow('Magnesite Conveyor Rendezvous', annotated)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            rclpy.shutdown()
        elif key == ord('r'):
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
            self.get_logger().info(
                f"Resolution increased to {self.inference_size}")
        elif key in [ord('d'), ord('D')]:
            self.inference_size = max(self.inference_size - 64, 320)
            self.get_logger().info(
                f"Resolution decreased to {self.inference_size}")
        elif key == ord('y'):
            self.y_park = round(self.y_park + 0.005, 3)
            self.solver.y_park = self.y_park
            self.get_logger().info(f"Y_PARK = {self.y_park:+.3f} m")
        elif key == ord('g'):
            self.y_park = round(self.y_park - 0.005, 3)
            self.solver.y_park = self.y_park
            self.get_logger().info(f"Y_PARK = {self.y_park:+.3f} m")
        elif key == ord('o'):
            self.solver.planning_overhead = round(
                self.solver.planning_overhead + 0.5, 1)
            self.get_logger().info(
                f"Planning overhead = {self.solver.planning_overhead:.1f}s  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('p'):
            self.solver.planning_overhead = max(
                0.0, round(self.solver.planning_overhead - 0.5, 1))
            self.get_logger().info(
                f"Planning overhead = {self.solver.planning_overhead:.1f}s  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('n'):
            self.solver.nominal_motion = round(
                self.solver.nominal_motion + 0.05, 2)
            self.get_logger().info(
                f"Nominal motion = {self.solver.nominal_motion:.2f}s  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('m'):
            self.solver.nominal_motion = max(
                0.0, round(self.solver.nominal_motion - 0.05, 2))
            self.get_logger().info(
                f"Nominal motion = {self.solver.nominal_motion:.2f}s  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('k'):
            self.solver.robot_vel_scale = min(
                1.0, round(self.solver.robot_vel_scale + 0.01, 2))
            self.get_logger().info(
                f"Robot vel scale = {self.solver.robot_vel_scale:.0%}  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('j'):
            self.solver.robot_vel_scale = max(
                0.01, round(self.solver.robot_vel_scale - 0.01, 2))
            self.get_logger().info(
                f"Robot vel scale = {self.solver.robot_vel_scale:.0%}  "
                f"→ exec_t = {self.solver._calc_exec_time():.1f}s")
        elif key == ord('f'):
            # Manual fire
            if tracked_detections is not None and len(tracked_detections) > 0:
                for i_, tid in enumerate(tracked_detections.tracker_id):
                    if (tracked_detections.class_id[i_] == 1
                            and tid in self.track_xz_smooth):
                        s = self.track_xz_smooth[tid]
                        self.publish_target_base_link(
                            s['x'], self.y_park, s['z'], tid)
                        self.triggered_ids.add(tid)
                        break
        elif key == ord('h'):
            print(
                "Controls:\n"
                "  +/-  Zoom | i/d  Resolution | r/v  Draw lines\n"
                "  y/g  Y_PARK (+/-5mm)\n"
                "  o/p  Planning overhead (+/-0.5s)\n"
                "  n/m  Nominal motion time at 100% (+/-0.05s)\n"
                "  k/j  Robot velocity scale (+/-1%)\n"
                "  f    Force-fire first MAG rock (test)\n"
                "  q    Quit")

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def publish_target_base_link(self, x, y, z, tracker_id):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Published target ID {tracker_id} [base_link]: "
            f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}")


# ===================================================================
# Main
# ===================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MagnesiteConveyorRendezvous()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
