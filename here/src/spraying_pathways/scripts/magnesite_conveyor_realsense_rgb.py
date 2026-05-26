#!/usr/bin/env python3
"""
magnesite_conveyor_realsense_rgb.py

Same push logic as magnesite_conveyor_rendezvous.py but opens the
RealSense camera DIRECTLY via pyrealsense2 — colour stream only,
no depth stream, no ROS realsense2_camera driver required.

Intrinsics are read straight from the SDK colour stream profile.
3D position is estimated by assuming the belt sits at a fixed known
distance (BELT_DEPTH_M).  Camera pose relative to robot base_link is
configured manually with CAM_TRANSLATION / CAM_ROTATION_RPY (same
approach as magnesite_conveyor_usb_cam.py).

Keys (OpenCV window must have focus – click on it first):
  z / Z   push_execute_lead  +0.05 s
  x / X   push_execute_lead  -0.05 s
  y / Y   Y_PARK             +5 mm
  g / G   Y_PARK             -5 mm
  r       draw trigger line
  v       draw verify line
  a       draw analysis ROI (click 2 corners)
  A       clear ROI (full frame inference)
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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float64, Bool

import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time
from typing import Optional, Tuple

from filterpy.kalman import KalmanFilter

from conveyor_rendezvous_solver import ConveyorRendezvousSolver

# ============================================================
# RealSense colour stream settings
# ============================================================
RS_WIDTH  = 1280
RS_HEIGHT = 720
RS_FPS    = 30

# ============================================================
# Camera pose in robot base_link frame.
# CAM_TRANSLATION = [x, y, z] metres from base_link origin to camera.
# CAM_ROTATION_RPY = [roll, pitch, yaw] in DEGREES (camera → base_link).
# Adjust to match your physical setup.
# ============================================================
CAM_TRANSLATION  = [0.0, 0.255, 0.60]
CAM_ROTATION_RPY = [180.0, 0.0, 0.0]

# Distance from camera optical centre to belt surface (metres).
BELT_DEPTH_M = 0.60

# ============================================================
# Model / detection
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

# ============================================================
# Robot / push geometry
# ============================================================
Y_PARK = -0.099

V_ROBOT_CARTESIAN    = 0.035
PLANNING_OVERHEAD_S  = 0.13
NOMINAL_MOTION_S     = 0.70
ROBOT_VELOCITY_SCALE = 0.40
T_PREP_S             = 0.7

WORKSPACE_Y_MIN = -0.15
WORKSPACE_Y_MAX =  0.35
TOOL_WIDTH_Y    =  0.06

PUSH_EXECUTE_LEAD_S = 2.40

# Kalman Filter
KF_MEASUREMENT_NOISE_R = 0.008
KF_PROCESS_NOISE_POS_Q = 0.001
KF_PROCESS_NOISE_VEL_Q = 0.05

XZ_EMA_ALPHA = 0.3

BELT_Z        = 0.050
PUSH_Z_OFFSET = 0.001

RETRIGGER_COOLDOWN_S = 5.0
MIN_TRACK_AGE        = 3
MIN_BELT_SPEED       = 0.005


# ============================================================
# Coordinate helpers
# ============================================================
def _build_cam_to_base(translation, rpy_deg):
    r, p, y = [np.deg2rad(a) for a in rpy_deg]
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = translation
    return T


def pixel_to_base_link(px, py, depth_m, fx, fy, cx, cy, T_cam_base):
    x_cam = (px - cx) / fx * depth_m
    y_cam = (py - cy) / fy * depth_m
    z_cam = depth_m
    p_cam  = np.array([x_cam, y_cam, z_cam, 1.0])
    p_base = T_cam_base @ p_cam
    return float(p_base[0]), float(p_base[1]), float(p_base[2])


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
class MagnesiteConveyorRealsenseRgb(Node):
    def __init__(self):
        super().__init__('magnesite_conveyor_realsense_rgb')

        self.model        = YOLO(MODEL_PATH)
        self.byte_tracker = sv.ByteTrack()

        self.target_pub       = self.create_publisher(PointStamped, '/magnesite_target', 10)
        self.alert_pub        = self.create_publisher(String, '/magnesite_alerts', 10)
        self.pre_position_pub = self.create_publisher(Bool, '/magnesite_pre_position', 10)

        self.create_subscription(Float64, '/magnesite_exec_time',   self.exec_time_cb,   10)
        self.create_subscription(Float64, '/magnesite_goto_push_t', self.goto_push_t_cb, 10)

        # Camera → base_link transform (manual, no TF2 needed)
        self.T_cam_base = _build_cam_to_base(CAM_TRANSLATION, CAM_ROTATION_RPY)

        # Open RealSense — colour stream only
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, RS_WIDTH, RS_HEIGHT, rs.format.bgr8, RS_FPS)
        profile = self._pipeline.start(cfg)

        # Read intrinsics directly from SDK
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self._fx = intr.fx
        self._fy = intr.fy
        self._cx = intr.ppx
        self._cy = intr.ppy
        self.get_logger().info(
            f"RealSense colour opened: {RS_WIDTH}x{RS_HEIGHT}@{RS_FPS}fps  "
            f"fx={self._fx:.1f} fy={self._fy:.1f} cx={self._cx:.1f} cy={self._cy:.1f}")

        self.zoom           = ZOOM_FACTOR
        self.inference_size = INFERENCE_SIZE

        self.y_park            = Y_PARK
        self.push_execute_lead = PUSH_EXECUTE_LEAD_S

        # Track state
        self.track_kfs          : dict = {}
        self.track_last_t       : dict = {}
        self.track_age          : dict = {}
        self.track_xz_smooth    : dict = {}
        self.triggered_ids      : set  = set()
        self.last_trigger_time  : dict = {}
        self.armed_ids          : set  = set()
        self.pre_positioned_ids : set  = set()
        self.offscreen_timers   : dict = {}

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

        self.skip_frames     = SKIP_FRAMES
        self.frame_count     = 0
        self.last_detections = None
        self.drawing_mode    = None
        self.current_clicks  = []
        self._last_key_name  = ''
        self.roi_rect        = None
        self.inference_ms    = 0.0

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

        cv2.namedWindow('Magnesite RealSense RGB', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Magnesite RealSense RGB', self.DISPLAY_W, self.DISPLAY_H)
        cv2.setMouseCallback('Magnesite RealSense RGB', self.mouse_callback)

        # Drive the main loop from a ROS timer (~30 Hz)
        self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info(
            f"Node ready. belt_depth={BELT_DEPTH_M}m Y_PARK={self.y_park:.3f}m "
            f"push_lead={self.push_execute_lead:.3f}s")

    # ----------------------------------------------------------------
    # 3D projection
    # ----------------------------------------------------------------
    def _pixel_to_base_link(self, px: float, py: float) -> Tuple[float, float, float]:
        return pixel_to_base_link(px, py, BELT_DEPTH_M,
                                  self._fx, self._fy, self._cx, self._cy,
                                  self.T_cam_base)

    def _get_edge_x_base_link(self, idx, dets, zf, crop_rect) -> Optional[Tuple]:
        def z2cam(zx, zy):
            ox, oy = zoomed_pixel_to_original(zx, zy, self.zoom, crop_rect,
                                              self.DISPLAY_W, self.DISPLAY_H)
            cx_ = ox * RS_WIDTH  / self.DISPLAY_W
            cy_ = oy * RS_HEIGHT / self.DISPLAY_H
            return cx_, cy_

        has_mask = (dets.mask is not None
                    and idx < len(dets.mask)
                    and dets.mask[idx] is not None)

        if has_mask:
            mask = dets.mask[idx]
            mys, mxs = np.where(mask > 0)
            if len(mxs) == 0:
                has_mask = False
            else:
                nl_x, nl_y = z2cam(mxs[mxs.argmin()], mys[mxs.argmin()])
                nr_x, nr_y = z2cam(mxs[mxs.argmax()], mys[mxs.argmax()])
                nc_x, nc_y = z2cam(float(np.mean(mxs)), float(np.mean(mys)))
                bll = self._pixel_to_base_link(nl_x, nl_y)
                blr = self._pixel_to_base_link(nr_x, nr_y)
                blc = self._pixel_to_base_link(nc_x, nc_y)
                return min(bll[0], blr[0]), blc[1], blc[2]

        if not has_mask:
            zx1, zy1, zx2, zy2 = dets.xyxy[idx]
            zcy = (zy1 + zy2) / 2.0
            nl_x, nl_y = z2cam(zx1, zcy)
            nr_x, nr_y = z2cam(zx2, zcy)
            nc_x, nc_y = z2cam((zx1 + zx2) / 2.0, zcy)
            bll = self._pixel_to_base_link(nl_x, nl_y)
            blr = self._pixel_to_base_link(nr_x, nr_y)
            blc = self._pixel_to_base_link(nc_x, nc_y)
            return min(bll[0], blr[0]), blc[1], blc[2]

        return None

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
    # Feedback callbacks
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
        kf   = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.array([[y0], [0.0]])
        kf.F = np.array([[1.0, 1.0], [0.0, 1.0]])
        kf.H = np.array([[1.0, 0.0]])
        kf.P = np.array([[0.05, 0.0], [0.0, 0.5]])
        kf.R = np.array([[KF_MEASUREMENT_NOISE_R]])
        kf.Q = np.array([[KF_PROCESS_NOISE_POS_Q, 0.0],
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
    # Main loop (timer-driven, ~30 Hz)
    # ----------------------------------------------------------------
    def loop(self):
        frames = self._pipeline.poll_for_frames()
        if not frames:
            return
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        full_frame = np.asanyarray(color_frame.get_data())  # already BGR (bgr8 format)

        self.fps_monitor.start()
        self.frame_count += 1

        full_frame = cv2.resize(full_frame, (self.DISPLAY_W, self.DISPLAY_H))
        zf, crop_rect = center_zoom(full_frame, self.zoom)

        run_infer = (self.frame_count % self.skip_frames == 0) or self.last_detections is None

        if run_infer:
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
            self.inference_ms = 0.0
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

                if (tr_in[i] or tr_out[i]) and cid == 1:
                    if tid not in self.armed_ids:
                        self.armed_ids.add(tid)
                        self.get_logger().info(f"[ARMED] ID {tid}")

                if cid != 1:
                    continue

                er = self._get_edge_x_base_link(i, tracked, zf, crop_rect)
                if er is None:
                    continue

                edge_x, bl_y, bl_z = er
                xs, zs             = self._smooth_xz(tid, edge_x, bl_z)
                y_kf, vy_kf        = self._update_kf(tid, t_now, bl_y)

                age         = self.track_age.get(tid, 0)
                last_t      = self.last_trigger_time.get(tid, 0.0)
                cooldown_ok = (t_now - last_t) > RETRIGGER_COOLDOWN_S

                if (tid not in self.armed_ids
                        or tid in self.triggered_ids
                        or age < MIN_TRACK_AGE
                        or vy_kf >= -MIN_BELT_SPEED
                        or not cooldown_ok):
                    if v_in[i] or v_out[i]:
                        self._handle_verify_crossed(tid)
                    continue

                time_to_park = (y_kf - self.y_park) / abs(vy_kf)

                if tid not in self.pre_positioned_ids:
                    self.pre_position_pub.publish(Bool(data=True))
                    self.pre_positioned_ids.add(tid)
                    self.get_logger().info(
                        f"[PRE-POS] ID {tid}: y={y_kf:.3f}m "
                        f"vy={vy_kf*100:.1f}cm/s t_to_park={time_to_park:.2f}s")

                t_lat = (self.measured_goto_push_t + PLANNING_OVERHEAD_S
                         if self.measured_goto_push_t is not None
                         else self.push_execute_lead)

                if tid in self.pre_positioned_ids and time_to_park <= t_lat:
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

        # Purge stale tracks
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
                    vy_d = float(self.track_kfs[tid].x[1, 0])
                    y_d  = float(self.track_kfs[tid].x[0, 0])
                    if cid == 1 and abs(vy_d) > 1e-5:
                        t2p   = (y_d - self.y_park) / abs(vy_d)
                        extra = f" t2p={t2p:.1f}s"
                        if tid in self.triggered_ids:          extra += ' [PUSHED]'
                        elif tid in self.pre_positioned_ids:   extra += ' [WAIT]'
                        elif tid in self.armed_ids:            extra += ' [ARMED]'
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

        if self.roi_rect is not None:
            rx1, ry1, rx2, ry2 = self.roi_rect
            cv2.rectangle(ann, (rx1, ry1), (rx2, ry2), (0, 165, 255), 2)
            cv2.putText(ann, "ANALYSIS ROI", (rx1 + 5, ry1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        if self.drawing_mode:
            if self.drawing_mode == 'ROI':
                cv2.putText(ann, "CLICK 2 CORNERS FOR ANALYSIS ROI",
                    (self.DISPLAY_W//2-320, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
            else:
                cv2.putText(ann, f"CLICK 2 POINTS FOR {self.drawing_mode} LINE",
                    (self.DISPLAY_W//2-300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            for c in self.current_clicks:
                cv2.circle(ann, c, 5, (255, 255, 255), -1)

        ann[5:130, 5:660] = (ann[5:130, 5:660] * 0.4).astype(np.uint8)
        self.fps_monitor.stop()
        cv2.putText(ann,
            f"FPS:{self.fps_monitor.getFPS():.0f} | "
            f"Infer:{self.inference_ms:.0f}ms | "
            f"Res:{self.inference_size} | ZOOM:{self.zoom:.1f}x | RS-RGB depth={BELT_DEPTH_M:.2f}m",
            (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2)
        t_lat_hud = (self.measured_goto_push_t + PLANNING_OVERHEAD_S
                     if self.measured_goto_push_t is not None
                     else self.push_execute_lead)
        cv2.putText(ann,
            f"EE_Y={self.y_park:+.4f}m [y/g]   t_lat={t_lat_hud:.3f}s [z/x fallback]   "
            f"depth={BELT_DEPTH_M:.2f}m",
            (15, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 2)
        cv2.putText(ann,
            f"Fire when t2p <= t_lat | goto_push_t={self.measured_goto_push_t or 'n/a'}",
            (15, 79), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 80), 1)
        if self.offscreen_timers:
            cv2.putText(ann,
                f"OFF-SCREEN PENDING: {len(self.offscreen_timers)}",
                (self.DISPLAY_W - 310, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
        cv2.putText(ann,
            f"Last key: [{self._last_key_name}]  (click window to capture keys)",
            (15, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 0), 1)

        cv2.imshow('Magnesite RealSense RGB', ann)

        # ============================================================
        # KEY HANDLING
        # ============================================================
        raw = cv2.waitKey(1)
        key = raw & 0xFF

        if key in (0xFF, 255):
            return

        try:
            self._last_key_name = chr(key)
        except ValueError:
            self._last_key_name = f"#{key}"

        if key in (ord('q'), 27):
            self._pipeline.stop()
            cv2.destroyAllWindows()
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

    def destroy_node(self):
        self._pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = MagnesiteConveyorRealsenseRgb()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
