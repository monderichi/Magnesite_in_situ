#!/usr/bin/env python3
"""
magnesite_conveyor_rendezvous.py

Two-stage push timing:
  Stage 1 - Pre-position: fired IMMEDIATELY when a magnesite rock is armed
             (crossed trigger line) and has a stable KF velocity estimate.
             Robot moves to push-start and WAITS there.
  Stage 2 - Push: fired when KF predicts rock reaches Y_PARK in
             push_execute_lead seconds.  Robot is already in position
             so only network + MoveIt startup latency matters.

Sign convention (base_link frame):
  vy_kf < 0  ->  rock moving in -Y direction  ->  approaching EE  (correct)
  time_to_park = (y_kf - Y_PARK) / |vy_kf|   (positive when rock upstream)

Camera is 255 mm from robot base in Y  ->  Y_PARK = 0.255 m

Keys (OpenCV window must have focus - click on it first):
  z / Z   push_execute_lead  +0.05 s
  x / X   push_execute_lead  -0.05 s
  y / Y   Y_PARK             +5 mm
  g / G   Y_PARK             -5 mm
  r       draw trigger line
  v       draw verify line
  + / =   zoom in
  - / _   zoom out
  i / I   inference size up
  d / D   inference size down
  f       force-fire first MAG rock (test)
  h       show full key list
  q / ESC quit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float64, Bool
import message_filters
from cv_bridge import CvBridge

import cv2
import image_geometry
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time
from typing import Optional, Tuple

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from filterpy.kalman import KalmanFilter

from conveyor_rendezvous_solver import ConveyorRendezvousSolver

# ============================================================
# Configuration
# ============================================================
MODEL_PATH = (
    "/media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/"
    "yolo_uv/only_rock.pt"
)
MAGNESITE_THRESHOLD = 50.0
CONFIDENCE          = 0.35
INFERENCE_SIZE      = 640
MIN_MASK_PIXELS     = 200
BLUR_KERNEL         = (5, 5)
USE_HALF            = False
SKIP_FRAMES         = 1
ZOOM_FACTOR         = 2.0

# EE Y position at push-start, computed via FK of park_joints_ in magnesite_pusher.cpp.
# Push start (FK): X=-0.229, Y=-0.099, Z=0.118  (base_link frame).
# This is NOT the camera distance — it is the actual intercept Y the rock must reach.
Y_PARK = -0.099

# Robot dynamics
V_ROBOT_CARTESIAN   = 0.035
PLANNING_OVERHEAD_S = 0.13   # Cartesian planning overhead after goPushStart completes
NOMINAL_MOTION_S    = 0.70
ROBOT_VELOCITY_SCALE = 0.40
T_PREP_S             = 0.7

WORKSPACE_Y_MIN = -0.15
WORKSPACE_Y_MAX =  0.35
TOOL_WIDTH_Y    =  0.06

# Stage 2 lead: full latency from Python publish → EE begins push motion.
# = goPushStart travel time (~2.27s measured) + Cartesian planning (~0.13s).
# After the first cycle, self.measured_goto_push_t replaces the static portion.
PUSH_EXECUTE_LEAD_S = 2.40

# Kalman Filter
KF_MEASUREMENT_NOISE_R  = 0.008
KF_PROCESS_NOISE_POS_Q  = 0.001
KF_PROCESS_NOISE_VEL_Q  = 0.05

XZ_EMA_ALPHA = 0.3

BELT_Z         = 0.050
PUSH_Z_OFFSET  = 0.001

RETRIGGER_COOLDOWN_S = 5.0
MIN_TRACK_AGE        = 3
MIN_BELT_SPEED       = 0.005


# ============================================================
# Helpers
# ============================================================
def center_zoom(frame, zoom):
    h, w = frame.shape[:2]
    cw = int(w / zoom); ch = int(h / zoom)
    x1 = (w - cw) // 2;  y1 = (h - ch) // 2
    zoomed = cv2.resize(frame[y1:y1+ch, x1:x1+cw], (w, h),
                        interpolation=cv2.INTER_LINEAR)
    return zoomed, (x1, y1, x1+cw, y1+ch)


def zoomed_pixel_to_original(zx, zy, zoom, crop_rect, fw, fh):
    x1, y1, x2, y2 = crop_rect
    return int((zx / fw) * (x2-x1) + x1), int((zy / fh) * (y2-y1) + y1)


def classify_rock_lab_otsu(frame, rock_mask):
    total = np.count_nonzero(rock_mask)
    if total < MIN_MASK_PIXELS:
        return False, 0, 0, 0, 0
    ys, xs = np.where(rock_mask > 0)
    y1, y2 = ys.min(), ys.max() + 1
    x1, x2 = xs.min(), xs.max() + 1
    lab   = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2LAB)
    lb    = cv2.GaussianBlur(lab[:, :, 0], BLUR_KERNEL, 0)
    tv, binary = cv2.threshold(lb, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    rm    = rock_mask[y1:y2, x1:x2] > 0
    bb    = binary > 0
    wc    = int(np.count_nonzero(rm & bb))
    dc    = int(np.count_nonzero(rm & ~bb))
    wp    = wc / total * 100
    dp    = dc / total * 100
    ratio = wc / dc if dc > 0 else float('inf')
    return wp > MAGNESITE_THRESHOLD, wp, dp, ratio, tv


# ============================================================
# Node
# ============================================================
class MagnesiteConveyorRendezvous(Node):
    def __init__(self):
        super().__init__('magnesite_conveyor_rendezvous')
        self.bridge       = CvBridge()
        self.model        = YOLO(MODEL_PATH)
        self.byte_tracker = sv.ByteTrack()

        self.target_pub       = self.create_publisher(PointStamped, '/magnesite_target', 10)
        self.alert_pub        = self.create_publisher(String, '/magnesite_alerts', 10)
        self.pre_position_pub = self.create_publisher(Bool, '/magnesite_pre_position', 10)

        self.cam_model    = image_geometry.PinholeCameraModel()
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.cam_info_cb, 10)
        self.intrinsics_loaded = False

        sc = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        sd = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        ts = message_filters.ApproximateTimeSynchronizer([sc, sd], 10, 0.1)
        ts.registerCallback(self.sync_callback)

        self.zoom           = ZOOM_FACTOR
        self.inference_size = INFERENCE_SIZE

        # Runtime-tunable
        self.y_park            = Y_PARK
        self.push_execute_lead = PUSH_EXECUTE_LEAD_S

        # Track state
        self.track_kfs         : dict = {}
        self.track_last_t      : dict = {}
        self.track_age         : dict = {}
        self.track_xz_smooth   : dict = {}
        self.triggered_ids     : set  = set()
        self.last_trigger_time : dict = {}
        self.armed_ids         : set  = set()
        self.pre_positioned_ids: set  = set()
        self.offscreen_timers  : dict = {}   # tid -> one-shot Timer for off-screen push

        self.solver = ConveyorRendezvousSolver(
            y_park=Y_PARK,
            v_robot=V_ROBOT_CARTESIAN,
            t_overhead=PLANNING_OVERHEAD_S,
            y_workspace_min=WORKSPACE_Y_MIN,
            y_workspace_max=WORKSPACE_Y_MAX,
            t_push=T_PREP_S,
            tool_width_y=TOOL_WIDTH_Y,
        )

        self.measured_exec_t      = None
        self.measured_goto_push_t = None
        self.create_subscription(Float64, '/magnesite_exec_time',   self.exec_time_cb, 10)
        self.create_subscription(Float64, '/magnesite_goto_push_t', self.goto_push_t_cb, 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.skip_frames     = SKIP_FRAMES
        self.frame_count     = 0
        self.last_detections = None
        self.drawing_mode    = None
        self.current_clicks  = []
        self._last_key_name  = ''   # echoed on HUD to verify window has focus
        self.roi_rect        = None  # (x1,y1,x2,y2) in display coords; None = full frame

        self.DISPLAY_W = 1280
        self.DISPLAY_H = 800

        self.TRIGGER_START = sv.Point(50, 400)
        self.TRIGGER_END   = sv.Point(self.DISPLAY_W - 50, 400)
        self.VERIFY_START  = sv.Point(50, 700)
        self.VERIFY_END    = sv.Point(self.DISPLAY_W - 50, 700)

        self.trigger_zone = sv.LineZone(start=self.TRIGGER_START, end=self.TRIGGER_END)
        self.verify_zone  = sv.LineZone(start=self.VERIFY_START,  end=self.VERIFY_END)

        palette = sv.ColorPalette(colors=[sv.Color(0, 200, 0), sv.Color(0, 100, 255)])
        self.mask_annotator    = sv.MaskAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS, opacity=0.40)
        self.polygon_annotator = sv.PolygonAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS, thickness=2)
        self.box_annotator     = sv.BoxCornerAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS)
        self.label_annotator   = sv.LabelAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS,
                                                   text_position=sv.Position.TOP_CENTER)
        self.trigger_annotator = sv.LineZoneAnnotator(color=sv.Color.RED,    thickness=2)
        self.verify_annotator  = sv.LineZoneAnnotator(color=sv.Color.YELLOW, thickness=2)
        self.fps_monitor = cv2.TickMeter()

        cv2.namedWindow('Magnesite Conveyor Rendezvous', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Magnesite Conveyor Rendezvous', self.DISPLAY_W, self.DISPLAY_H)
        cv2.setMouseCallback('Magnesite Conveyor Rendezvous', self.mouse_callback)

        self.get_logger().info(
            f"Node ready. Y_PARK={self.y_park:.3f}m "
            f"push_lead={self.push_execute_lead:.3f}s  "
            "[Stage1=immediate-on-arm | Stage2=time_to_park<=push_lead]")

    # ----------------------------------------------------------------
    # Camera
    # ----------------------------------------------------------------
    def cam_info_cb(self, msg):
        self.cam_model.fromCameraInfo(msg)
        self.intrinsics_loaded = True
        self.destroy_subscription(self.cam_info_sub)

    # ----------------------------------------------------------------
    # Mouse
    # ----------------------------------------------------------------
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.drawing_mode is not None:
            self.current_clicks.append((x, y))
            if len(self.current_clicks) == 2:
                p1 = sv.Point(self.current_clicks[0][0], self.current_clicks[0][1])
                p2 = sv.Point(self.current_clicks[1][0], self.current_clicks[1][1])
                if self.drawing_mode == 'TRIGGER':
                    self.TRIGGER_START, self.TRIGGER_END = p1, p2
                    self.trigger_zone = sv.LineZone(start=p1, end=p2)
                elif self.drawing_mode == 'VERIFY':
                    self.VERIFY_START, self.VERIFY_END = p1, p2
                    self.verify_zone = sv.LineZone(start=p1, end=p2)
                elif self.drawing_mode == 'ROI':
                    rx1 = min(self.current_clicks[0][0], self.current_clicks[1][0])
                    ry1 = min(self.current_clicks[0][1], self.current_clicks[1][1])
                    rx2 = max(self.current_clicks[0][0], self.current_clicks[1][0])
                    ry2 = max(self.current_clicks[0][1], self.current_clicks[1][1])
                    if rx2 - rx1 > 20 and ry2 - ry1 > 20:
                        self.roi_rect = (rx1, ry1, rx2, ry2)
                self.drawing_mode   = None
                self.current_clicks = []

    # ----------------------------------------------------------------
    # Depth helper
    # ----------------------------------------------------------------
    def get_valid_depth(self, depth_img, x, y, max_radius=5):
        h, w = depth_img.shape
        x, y = int(x), int(y)
        if 0 <= x < w and 0 <= y < h and depth_img[y, x] > 0:
            return float(depth_img[y, x])
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h and depth_img[ny, nx] > 0:
                        return float(depth_img[ny, nx])
        return 0.0

    # ----------------------------------------------------------------
    # TF2
    # ----------------------------------------------------------------
    def _to_base_link(self, xc, yc, zc, stamp) -> Optional[Tuple]:
        pt = PointStamped()
        pt.header.stamp    = stamp
        pt.header.frame_id = 'camera_color_optical_frame'
        pt.point.x, pt.point.y, pt.point.z = float(xc), float(yc), float(zc)
        try:
            t = self.tf_buffer.transform(
                pt, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1))
            return t.point.x, t.point.y, t.point.z
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=5.0)
            return None

    # ----------------------------------------------------------------
    # Feedback
    # ----------------------------------------------------------------
    def exec_time_cb(self, msg: Float64):
        self.measured_exec_t = float(msg.data)
        self.get_logger().info(f"exec_time = {self.measured_exec_t:.2f}s")

    def goto_push_t_cb(self, msg: Float64):
        self.measured_goto_push_t = float(msg.data)
        self.get_logger().info(f"goPushStart = {self.measured_goto_push_t:.3f}s")

    # ----------------------------------------------------------------
    # Kalman Filter
    # ----------------------------------------------------------------
    def _init_kf(self, y0: float) -> KalmanFilter:
        kf    = KalmanFilter(dim_x=2, dim_z=1)
        kf.x  = np.array([[y0], [0.0]])
        kf.F  = np.array([[1.0, 1.0], [0.0, 1.0]])
        kf.H  = np.array([[1.0, 0.0]])
        kf.P  = np.array([[0.05, 0.0], [0.0, 0.5]])
        kf.R  = np.array([[KF_MEASUREMENT_NOISE_R]])
        kf.Q  = np.array([[KF_PROCESS_NOISE_POS_Q, 0.0],
                          [0.0, KF_PROCESS_NOISE_VEL_Q]])
        return kf

    def _update_kf(self, tid: int, t: float, y_meas: float):
        if tid not in self.track_kfs:
            self.track_kfs[tid]    = self._init_kf(y_meas)
            self.track_last_t[tid] = t
            self.track_age[tid]    = 1
            return y_meas, 0.0
        kf = self.track_kfs[tid]
        dt = t - self.track_last_t[tid]
        if dt > 1e-4:
            kf.F[0, 1] = dt
            kf.predict()
            kf.update(np.array([[y_meas]]))
            self.track_last_t[tid] = t
            self.track_age[tid]   += 1
        return float(kf.x[0, 0]), float(kf.x[1, 0])

    def _smooth_xz(self, tid: int, xm: float, zm: float):
        if tid not in self.track_xz_smooth:
            self.track_xz_smooth[tid] = {'x': xm, 'z': zm}
            return xm, zm
        s = self.track_xz_smooth[tid]
        s['x'] = XZ_EMA_ALPHA * xm + (1 - XZ_EMA_ALPHA) * s['x']
        s['z'] = XZ_EMA_ALPHA * zm + (1 - XZ_EMA_ALPHA) * s['z']
        return s['x'], s['z']

    # ----------------------------------------------------------------
    # Edge detection
    # ----------------------------------------------------------------
    def _get_edge_x_base_link(self, idx, dets, zf, crop_rect,
                               depth_img, stamp) -> Optional[Tuple]:
        dfh, dfw = depth_img.shape

        def p2bl(px, py, dmm):
            if dmm <= 0: return None
            dm  = dmm / 1000.0
            ray = self.cam_model.projectPixelTo3dRay((px, py))
            return self._to_base_link(ray[0]*dm, ray[1]*dm, dm, stamp)

        def z2n(zx, zy):
            ox, oy = zoomed_pixel_to_original(zx, zy, self.zoom, crop_rect,
                                              self.DISPLAY_W, self.DISPLAY_H)
            return int(ox * dfw / self.DISPLAY_W), int(oy * dfh / self.DISPLAY_H)

        has_mask = (dets.mask is not None
                    and idx < len(dets.mask)
                    and dets.mask[idx] is not None)

        if has_mask:
            mask = dets.mask[idx]
            mys, mxs = np.where(mask > 0)
            if len(mxs) == 0:
                has_mask = False
            else:
                nl_x, nl_y = z2n(mxs[mxs.argmin()], mys[mxs.argmin()])
                nr_x, nr_y = z2n(mxs[mxs.argmax()], mys[mxs.argmax()])
                bll = p2bl(nl_x, nl_y, self.get_valid_depth(depth_img, nl_x, nl_y))
                blr = p2bl(nr_x, nr_y, self.get_valid_depth(depth_img, nr_x, nr_y))
                if bll is None or blr is None:
                    has_mask = False
                else:
                    cx, cy = int(np.mean(mxs)), int(np.mean(mys))
                    nc_x, nc_y = z2n(cx, cy)
                    blc = p2bl(nc_x, nc_y, self.get_valid_depth(depth_img, nc_x, nc_y))
                    if blc is None: return None
                    return min(bll[0], blr[0]), blc[1], blc[2]

        if not has_mask:
            zx1, zy1, zx2, zy2 = dets.xyxy[idx]
            zcy = (zy1 + zy2) / 2.0
            nl_x, nl_y = z2n(zx1, zcy)
            nr_x, nr_y = z2n(zx2, zcy)
            nc_x, nc_y = z2n((zx1+zx2)/2, zcy)
            bll = p2bl(nl_x, nl_y, self.get_valid_depth(depth_img, nl_x, nl_y))
            blr = p2bl(nr_x, nr_y, self.get_valid_depth(depth_img, nr_x, nr_y))
            blc = p2bl(nc_x, nc_y, self.get_valid_depth(depth_img, nc_x, nc_y))
            if bll is None or blr is None or blc is None: return None
            return min(bll[0], blr[0]), blc[1], blc[2]

        return None

    # ----------------------------------------------------------------
    # Main sync callback
    # ----------------------------------------------------------------
    def sync_callback(self, color_msg, depth_msg):
        if not self.intrinsics_loaded:
            return

        self.fps_monitor.start()
        self.frame_count += 1

        full_frame  = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img   = self.bridge.imgmsg_to_cv2(depth_msg,  desired_encoding='passthrough')
        full_frame  = cv2.resize(full_frame, (self.DISPLAY_W, self.DISPLAY_H))
        zf, crop_rect = center_zoom(full_frame, self.zoom)

        run_infer = (self.frame_count % self.skip_frames == 0) or self.last_detections is None

        if run_infer:
            # Crop to ROI for faster inference; map back after
            if self.roi_rect is not None:
                rx1, ry1, rx2, ry2 = self.roi_rect
                infer_frame = zf[ry1:ry2, rx1:rx2]
            else:
                infer_frame = zf

            t0 = time.perf_counter()
            results = self.model(infer_frame, verbose=False, conf=CONFIDENCE,
                                 imgsz=self.inference_size, half=USE_HALF)[0]
            self.inference_ms = (time.perf_counter() - t0) * 1000

            dets = sv.Detections.from_ultralytics(results)
            cids = []
            for i in range(len(dets)):
                if dets.mask is not None and i < len(dets.mask) and dets.mask[i] is not None:
                    rm = (dets.mask[i].astype(np.uint8)) * 255
                else:
                    bx1, by1, bx2, by2 = map(int, dets.xyxy[i])
                    rm = np.zeros(infer_frame.shape[:2], dtype=np.uint8)
                    rm[by1:by2, bx1:bx2] = 255
                is_mag, *_ = classify_rock_lab_otsu(infer_frame, rm)
                cids.append(1 if is_mag else 0)
            dets.class_id = np.array(cids, dtype=int)

            # Remap detections from infer_frame coordinates to full zf coordinates
            if self.roi_rect is not None and len(dets) > 0:
                dets.xyxy[:, [0, 2]] += rx1
                dets.xyxy[:, [1, 3]] += ry1
                if dets.mask is not None:
                    full_masks = np.zeros(
                        (len(dets.mask), zf.shape[0], zf.shape[1]),
                        dtype=dets.mask.dtype)
                    full_masks[:, ry1:ry2, rx1:rx2] = dets.mask
                    dets.mask = full_masks

            tracked              = self.byte_tracker.update_with_detections(dets)
            self.last_detections = tracked
            fresh = True
        else:
            tracked = self.last_detections
            self.inference_ms = 0
            fresh = False

        n          = len(tracked) if tracked else 0
        active_ids = set()
        t_now      = time.monotonic()

        if fresh and tracked and n > 0:
            v_in,  v_out  = self.verify_zone.trigger(tracked)
            tr_in, tr_out = self.trigger_zone.trigger(tracked)

            for i in range(n):
                tid = tracked.tracker_id[i]
                cid = tracked.class_id[i]
                active_ids.add(tid)

                # Arm on trigger line crossing
                if (tr_in[i] or tr_out[i]) and cid == 1:
                    if tid not in self.armed_ids:
                        self.armed_ids.add(tid)
                        self.get_logger().info(f"[ARMED] ID {tid}")

                if cid != 1:
                    continue

                er = self._get_edge_x_base_link(
                    i, tracked, zf, crop_rect, depth_img,
                    self.get_clock().now().to_msg())
                if er is None:
                    continue

                edge_x, bl_y, bl_z = er
                xs, zs             = self._smooth_xz(tid, edge_x, bl_z)
                y_kf, vy_kf        = self._update_kf(tid, t_now, bl_y)

                age        = self.track_age.get(tid, 0)
                last_t     = self.last_trigger_time.get(tid, 0.0)
                cooldown_ok = (t_now - last_t) > RETRIGGER_COOLDOWN_S

                if (tid not in self.armed_ids
                        or tid in self.triggered_ids
                        or age < MIN_TRACK_AGE
                        or vy_kf >= -MIN_BELT_SPEED
                        or not cooldown_ok):
                    if v_in[i] or v_out[i]:
                        self._handle_verify_crossed(tid)
                    continue

                # time_to_park = seconds until rock reaches EE_Y (push-start Y).
                # Valid only when vy_kf < 0 (rock approaching) and y_kf > EE_Y.
                time_to_park = (y_kf - self.y_park) / abs(vy_kf)

                # ---- STAGE 1: signal robot to pre-position (informational) ----
                if tid not in self.pre_positioned_ids:
                    self.pre_position_pub.publish(Bool(data=True))
                    self.pre_positioned_ids.add(tid)
                    self.get_logger().info(
                        f"[PRE-POS] ID {tid}: y={y_kf:.3f}m "
                        f"vy={vy_kf*100:.1f}cm/s t_to_park={time_to_park:.2f}s")

                # ---- STAGE 2: fire push when rock is exactly T_lat away from EE_Y ----
                # T_lat = goPushStart travel time + Cartesian planning overhead.
                # Use the measured goto_push_t from the previous cycle when available,
                # otherwise fall back to the static PUSH_EXECUTE_LEAD_S constant.
                t_lat = (self.measured_goto_push_t + PLANNING_OVERHEAD_S
                         if self.measured_goto_push_t is not None
                         else self.push_execute_lead)

                if (tid in self.pre_positioned_ids
                        and time_to_park <= t_lat):
                    # Predicted rock Y when push actually starts (T_lat seconds from now).
                    y_at_push = y_kf + vy_kf * t_lat
                    if WORKSPACE_Y_MIN <= y_at_push <= WORKSPACE_Y_MAX:
                        self.publish_target(xs, self.y_park, BELT_Z + PUSH_Z_OFFSET, tid)
                        self.triggered_ids.add(tid)
                        self.last_trigger_time[tid] = t_now
                        self.get_logger().info(
                            f"[PUSH] ID {tid}: FIRE y={y_kf:.3f} "
                            f"t_to_park={time_to_park:.4f}s t_lat={t_lat:.3f}s "
                            f"y_intercept={y_at_push:.3f}m vy={vy_kf*100:.1f}cm/s")
                    else:
                        self.get_logger().warn(
                            f"[PUSH] ID {tid}: y_at_push={y_at_push:.3f} outside workspace, skip.")

                if v_in[i] or v_out[i]:
                    self._handle_verify_crossed(tid)

        # Purge stale tracks; schedule off-screen push for armed-but-unfired rocks
        for tid in [t for t in self.track_kfs if t not in active_ids]:
            if (tid in self.armed_ids
                    and tid not in self.triggered_ids
                    and tid not in self.offscreen_timers
                    and tid in self.track_kfs):
                kf    = self.track_kfs[tid]
                y_kf  = float(kf.x[0, 0])
                vy_kf = float(kf.x[1, 0])
                if vy_kf < -MIN_BELT_SPEED and y_kf > self.y_park:
                    t_lat = (self.measured_goto_push_t + PLANNING_OVERHEAD_S
                             if self.measured_goto_push_t is not None
                             else self.push_execute_lead)
                    # Propagate KF forward by time elapsed since last update
                    t_elapsed    = t_now - self.track_last_t.get(tid, t_now)
                    y_extrap     = y_kf + vy_kf * t_elapsed
                    time_to_park = (y_extrap - self.y_park) / abs(vy_kf)
                    fire_delay   = time_to_park - t_lat
                    xs = self.track_xz_smooth.get(tid, {}).get('x', -0.229)
                    if fire_delay > 0.05:
                        timer = self.create_timer(
                            fire_delay,
                            lambda tid_=tid, xs_=xs: self._fire_offscreen(tid_, xs_))
                        self.offscreen_timers[tid] = timer
                        self.get_logger().info(
                            f"[OFF-SCREEN] ID {tid}: left frame "
                            f"y={y_extrap:.3f}m vy={vy_kf*100:.1f}cm/s "
                            f"→ fire in {fire_delay:.3f}s")
                    else:
                        self.get_logger().warn(
                            f"[OFF-SCREEN] ID {tid}: too late "
                            f"(fire_delay={fire_delay:.3f}s), skip.")

            for d in (self.track_kfs, self.track_last_t, self.track_age,
                      self.track_xz_smooth, self.last_trigger_time):
                d.pop(tid, None)
            self.triggered_ids.discard(tid)
            self.armed_ids.discard(tid)
            self.pre_positioned_ids.discard(tid)

        # ============================================================
        # VISUALIZATION
        # ============================================================
        ann = zf.copy()

        if n > 0:
            if tracked.mask is not None:
                ann = self.mask_annotator.annotate(ann, tracked)
                ann = self.polygon_annotator.annotate(ann, tracked)
            ann = self.box_annotator.annotate(ann, tracked)
            labels = []
            for tid, cid in zip(tracked.tracker_id, tracked.class_id):
                vy_d = 0.0; extra = ''
                if tid in self.track_kfs:
                    vy_d  = float(self.track_kfs[tid].x[1, 0])
                    y_d   = float(self.track_kfs[tid].x[0, 0])
                    if cid == 1 and abs(vy_d) > 1e-5:
                        t2p   = (y_d - self.y_park) / abs(vy_d)
                        extra = f" t2p={t2p:.1f}s"
                        if tid in self.triggered_ids:      extra += ' [PUSHED]'
                        elif tid in self.pre_positioned_ids: extra += ' [WAIT]'
                        elif tid in self.armed_ids:          extra += ' [ARMED]'
                spd = f" {vy_d*100:.0f}cm/s" if abs(vy_d) > 0.001 else ''
                labels.append(f"ID:{tid} {'MAG' if cid==1 else 'ROCK'}{spd}{extra}")
            ann = self.label_annotator.annotate(ann, tracked, labels)

        ann = self.trigger_annotator.annotate(ann, line_counter=self.trigger_zone)
        ann = self.verify_annotator.annotate(ann,  line_counter=self.verify_zone)

        cv2.putText(ann, "TRIGGER (arm)",
            (int(self.TRIGGER_START.x), int(self.TRIGGER_START.y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(ann, "VERIFY",
            (int(self.VERIFY_START.x), int(self.VERIFY_START.y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Draw ROI rectangle (orange)
        if self.roi_rect is not None:
            rx1, ry1, rx2, ry2 = self.roi_rect
            cv2.rectangle(ann, (rx1, ry1), (rx2, ry2), (0, 165, 255), 2)
            cv2.putText(ann, "ANALYSIS ROI", (rx1 + 5, ry1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        if self.drawing_mode:
            if self.drawing_mode == 'ROI':
                cv2.putText(ann, "CLICK 2 CORNERS FOR ANALYSIS ROI",
                    (self.DISPLAY_W//2-320, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,165,255), 3)
            else:
                cv2.putText(ann, f"CLICK 2 POINTS FOR {self.drawing_mode} LINE",
                    (self.DISPLAY_W//2-300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
            for c in self.current_clicks:
                cv2.circle(ann, c, 5, (255,255,255), -1)

        # HUD panel
        ann[5:130, 5:620] = (ann[5:130, 5:620] * 0.4).astype(np.uint8)
        self.fps_monitor.stop()
        cv2.putText(ann,
            f"FPS:{self.fps_monitor.getFPS():.0f} | "
            f"Infer:{getattr(self,'inference_ms',0):.0f}ms | "
            f"Res:{self.inference_size} | ZOOM:{self.zoom:.1f}x",
            (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2)
        t_lat_hud = (self.measured_goto_push_t + PLANNING_OVERHEAD_S
                     if self.measured_goto_push_t is not None
                     else self.push_execute_lead)
        cv2.putText(ann,
            f"EE_Y={self.y_park:+.4f}m [y/g]   t_lat={t_lat_hud:.3f}s (goto+plan) [z/x fallback]",
            (15, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 2)
        cv2.putText(ann,
            f"Fire when t2p <= t_lat | goto_push_t={self.measured_goto_push_t or 'n/a'}",
            (15, 79), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 80), 1)
        if self.offscreen_timers:
            cv2.putText(ann,
                f"OFF-SCREEN PENDING: {len(self.offscreen_timers)}",
                (self.DISPLAY_W - 310, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
        # Key echo: if this doesn't update when you press a key the window has no focus
        cv2.putText(ann,
            f"Last key pressed: [{self._last_key_name}]  <- must update on keypress (click window)",
            (15, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 0), 1)

        cv2.imshow('Magnesite Conveyor Rendezvous', ann)

        # ============================================================
        # KEY HANDLING
        # The OpenCV window MUST have mouse focus for waitKey to work.
        # Click the window once if keys are not responding.
        # Both lower and upper case accepted.
        # ============================================================
        raw = cv2.waitKey(1)
        key = raw & 0xFF

        if key in (0xFF, 255):  # no key
            return

        try:
            self._last_key_name = chr(key)
        except ValueError:
            self._last_key_name = f"#{key}"

        if key in (ord('q'), 27):
            rclpy.shutdown()

        elif key == ord('r'):
            self.drawing_mode = 'TRIGGER'; self.current_clicks = []

        elif key == ord('v'):
            self.drawing_mode = 'VERIFY'; self.current_clicks = []

        elif key == ord('a'):
            self.drawing_mode = 'ROI'; self.current_clicks = []

        elif key == ord('A'):
            self.roi_rect = None
            self.get_logger().info("ROI cleared — full frame inference")

        elif key in (ord('+'), ord('=')):
            self.zoom = min(self.zoom + 0.25, 8.0)

        elif key in (ord('-'), ord('_')):
            self.zoom = max(self.zoom - 0.25, 1.0)

        elif key in (ord('i'), ord('I')):
            self.inference_size = min(self.inference_size + 64, 1280)
            self.get_logger().info(f"Inference size -> {self.inference_size}")

        elif key in (ord('d'), ord('D')):
            self.inference_size = max(self.inference_size - 64, 320)
            self.get_logger().info(f"Inference size -> {self.inference_size}")

        elif key in (ord('y'), ord('Y')):
            self.y_park = round(self.y_park + 0.005, 4)
            self.solver.y_park = self.y_park
            self.get_logger().info(f"Y_PARK -> {self.y_park:+.4f} m")

        elif key in (ord('g'), ord('G')):
            self.y_park = round(self.y_park - 0.005, 4)
            self.solver.y_park = self.y_park
            self.get_logger().info(f"Y_PARK -> {self.y_park:+.4f} m")

        elif key in (ord('z'), ord('Z')):
            self.push_execute_lead = round(self.push_execute_lead + 0.05, 3)
            self.get_logger().info(f"push_execute_lead -> {self.push_execute_lead:.3f}s")

        elif key in (ord('x'), ord('X')):
            self.push_execute_lead = max(0.0, round(self.push_execute_lead - 0.05, 3))
            self.get_logger().info(f"push_execute_lead -> {self.push_execute_lead:.3f}s")

        elif key == ord('f'):
            if tracked is not None and len(tracked) > 0:
                for i_, tid in enumerate(tracked.tracker_id):
                    if tracked.class_id[i_] == 1 and tid in self.track_xz_smooth:
                        s = self.track_xz_smooth[tid]
                        self.publish_target(s['x'], self.y_park,
                                            BELT_Z + PUSH_Z_OFFSET, tid)
                        self.triggered_ids.add(tid)
                        self.get_logger().info(f"[FORCE-FIRE] ID {tid}")
                        break

        elif key in (ord('h'), ord('H')):
            print(
                "\nControls (click OpenCV window first):\n"
                "  z/Z   push_execute_lead +0.05s\n"
                "  x/X   push_execute_lead -0.05s\n"
                "  y/Y   Y_PARK +5mm\n"
                "  g/G   Y_PARK -5mm\n"
                "  r     draw trigger line\n"
                "  v     draw verify line\n"
                "  a     draw analysis ROI (click 2 corners)\n"
                "  A     clear ROI (full frame inference)\n"
                "  +/-   zoom\n"
                "  i/d   inference resolution\n"
                "  f     force-fire\n"
                "  q     quit\n")

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _fire_offscreen(self, tid: int, xs: float):
        """One-shot timer callback: push a rock that has left the camera frame."""
        timer = self.offscreen_timers.pop(tid, None)
        if timer:
            timer.cancel()
        self.publish_target(xs, self.y_park, BELT_Z + PUSH_Z_OFFSET, tid)
        self.triggered_ids.add(tid)
        self.last_trigger_time[tid] = time.monotonic()
        self.get_logger().info(f"[OFF-SCREEN PUSH] ID {tid} fired.")

    def _handle_verify_crossed(self, tid: int):
        msg = f"ALERT: Magnesite ID {tid} passed Verification Line!"
        self.get_logger().warning(msg)
        self.alert_pub.publish(String(data=msg))
        self.triggered_ids.discard(tid)
        self.armed_ids.discard(tid)
        self.pre_positioned_ids.discard(tid)
        for d in (self.track_kfs, self.track_last_t, self.track_age, self.track_xz_smooth):
            d.pop(tid, None)

    def publish_target(self, x, y, z, tid):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Target ID {tid} -> X={x:.3f} Y={y:.3f} Z={z:.3f} [base_link]")


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = MagnesiteConveyorRendezvous()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()