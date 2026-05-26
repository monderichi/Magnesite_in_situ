#!/usr/bin/env python3
"""
Magnesite Conveyor Pipeline Node (ROS2 / MoveIt2 Integration)
Uses RealSense + YOLO + LAB L+Otsu + ByteTrack + Supervision LineZones.
Tracks rocks on a moving conveyor belt, and when a "Magnesite" rock crosses
the Trigger line, it calculates its exact 3D coordinates and publishes them to ROS2.
Then, if the rock crosses the Verification line further down, it alerts that 
the robot failed to remove the target.

Controls:
  q / ESC  - Quit
  s        - Screenshot
  +/-      - Increase/decrease zoom
  i/-      - Increase/decrease inference size
  h        - Help
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from ultralytics import YOLO
import supervision as sv
from datetime import datetime
import time
import math


# ============================================================
# Configuration
# ============================================================
MODEL_PATH          = "/media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/yolo_uv/only_rock.pt"
MAGNESITE_THRESHOLD = 50.0   # white pixels > 50% of total => MAGNESITE
CONFIDENCE          = 0.35
INFERENCE_SIZE      = 640
MIN_MASK_PIXELS     = 200
BLUR_KERNEL         = (5, 5)
USE_HALF            = False
SKIP_FRAMES         = 1
ZOOM_FACTOR         = 2.0    
DISPLAY_W           = 1280
DISPLAY_H           = 800

# Supervision Line Zone coordinates (relative to the zoomed 1280x800 frame)
# Trigger line is positioned earlier on the belt to give the arm time to move
TRIGGER_START       = sv.Point(50, 400)
TRIGGER_END         = sv.Point(DISPLAY_W - 50, 400)

# Verification line is further down the belt to check if rock was pushed off
VERIFY_START        = sv.Point(50, 700)
VERIFY_END          = sv.Point(DISPLAY_W - 50, 700)

# ------------------------------------------------------------
# Interactive Line Drawing State
# ------------------------------------------------------------
drawing_mode = None  # Can be 'TRIGGER', 'VERIFY', or None
current_clicks = []
trigger_zone = sv.LineZone(start=TRIGGER_START, end=TRIGGER_END)
verify_zone = sv.LineZone(start=VERIFY_START, end=VERIFY_END)

def mouse_callback(event, x, y, flags, param):
    global drawing_mode, current_clicks, TRIGGER_START, TRIGGER_END, VERIFY_START, VERIFY_END, trigger_zone, verify_zone
    if event == cv2.EVENT_LBUTTONDOWN and drawing_mode is not None:
        current_clicks.append((x, y))
        print(f"Click recorded at {x}, {y}")
        if len(current_clicks) == 2:
            p1 = sv.Point(current_clicks[0][0], current_clicks[0][1])
            p2 = sv.Point(current_clicks[1][0], current_clicks[1][1])
            if drawing_mode == 'TRIGGER':
                TRIGGER_START, TRIGGER_END = p1, p2
                trigger_zone = sv.LineZone(start=p1, end=p2)
                print(f"[*] Trigger Line updated: {p1} -> {p2}")
            elif drawing_mode == 'VERIFY':
                VERIFY_START, VERIFY_END = p1, p2
                verify_zone = sv.LineZone(start=p1, end=p2)
                print(f"[*] Verification Line updated: {p1} -> {p2}")
            drawing_mode = None
            current_clicks = []

def get_valid_depth(depth_frame, x, y, max_radius=5):
    """Fallback spatial search if the exact pixel has no depth (RealSense depth holes)."""
    depth = depth_frame.get_distance(int(x), int(y))
    if depth > 0:
        return depth
        
    w, h = depth_frame.get_width(), depth_frame.get_height()
    for r in range(1, max_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) == r or abs(dy) == r:
                    nx, ny = int(x + dx), int(y + dy)
                    if 0 <= nx < w and 0 <= ny < h:
                        depth = depth_frame.get_distance(nx, ny)
                        if depth > 0:
                            return depth
    return 0.0


# ============================================================
# ROS2 Node Definition
# ============================================================
class MagnesiteConveyorNode(Node):
    def __init__(self):
        super().__init__('magnesite_conveyor_tracker')
        self.get_logger().info('Magnesite Conveyor Tracker Node Started.')
        
        # Publisher for the target 3D coordinate (to be picked/pushed)
        self.target_pub = self.create_publisher(PointStamped, '/magnesite_target', 10)
        # Publisher for logs and alerts (e.g., verification line check)
        self.alert_pub = self.create_publisher(String, '/magnesite_alerts', 10)

    def publish_target(self, x, y, z, tracker_id):
        """
        Publishes the 3D coordinate of the magnesite that crossed the trigger line.
        """
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Ensure this matches your robot's camera TF frame
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published Mg Target [ID: {tracker_id}] -> X:{x:.4f}, Y:{y:.4f}, Z:{z:.4f}")

    def publish_alert(self, message):
        msg = String(data=message)
        self.alert_pub.publish(msg)
        self.get_logger().warning(message)  # .warn() deprecated in Humble, use .warning()


# ============================================================
# Center zoom crop
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
    zoomed  = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
    return zoomed, (x1, y1, x2, y2)

def zoomed_pixel_to_original(zx, zy, zoom, crop_rect, frame_w, frame_h):
    """
    Translates a pixel from the zoomed frame back to the original full-frame crop.
    """
    x1, y1, x2, y2 = crop_rect
    crop_w = x2 - x1
    crop_h = y2 - y1
    
    # Scale back to crop size
    orig_x = (zx / frame_w) * crop_w + x1
    orig_y = (zy / frame_h) * crop_h + y1
    
    return int(orig_x), int(orig_y)


# ============================================================
# LAB L+Otsu classifier
# ============================================================
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


def main():
    global drawing_mode, current_clicks
    # Initialize ROS2
    rclpy.init()
    ros_node = MagnesiteConveyorNode()

    print("=" * 60)
    print("MAGNESITE CONVEYOR PIPELINE (ROS2, TRACKING, ZONES)")
    print("=" * 60)

    # ----------------------------------------------------------
    # Load YOLO model
    # ----------------------------------------------------------
    print("Loading YOLO model...")
    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"  FAILED: {e}")
        return

    # ----------------------------------------------------------
    # Configure RealSense D455 — RGB and Depth
    # ----------------------------------------------------------
    print("Configuring RealSense D455...")
    pipeline = rs.pipeline()
    config   = rs.config()

    # Need both Color and Depth for 3D localization
    # NOTE: D455 depth sensor does NOT support 1280x800.
    # Use 848x480 for depth — rs.align() will remap it to the color frame correctly.
    config.enable_stream(rs.stream.color, DISPLAY_W, DISPLAY_H, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    # Align depth payload to color frame
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"  FAILED to start pipeline: {e}")
        return

    # Warm up camera
    print("Warming up camera...")
    for _ in range(10):
        pipeline.wait_for_frames()

    # Pre-warm YOLO
    print("Pre-warming YOLO model...")
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    if color_frame:
        dummy = np.asanyarray(color_frame.get_data())
        dummy_zoom, _ = center_zoom(dummy, ZOOM_FACTOR)
        model(dummy_zoom, verbose=False, conf=CONFIDENCE, imgsz=INFERENCE_SIZE, half=USE_HALF)
    
    # ----------------------------------------------------------
    # Supervision annotators / zones
    # ----------------------------------------------------------
    ROCK_COLOR = sv.Color(r=0,   g=200, b=0)
    MAG_COLOR  = sv.Color(r=0,   g=100, b=255)
    palette    = sv.ColorPalette(colors=[ROCK_COLOR, MAG_COLOR])

    mask_annotator    = sv.MaskAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS, opacity=0.40)
    polygon_annotator = sv.PolygonAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS, thickness=2)
    box_annotator     = sv.BoxCornerAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS)
    label_annotator   = sv.LabelAnnotator(color=palette, color_lookup=sv.ColorLookup.CLASS, text_position=sv.Position.TOP_CENTER)

    # Trackers
    byte_tracker      = sv.ByteTrack()

    # Line Zones
    trigger_annotator = sv.LineZoneAnnotator(color=sv.Color.RED, thickness=2, text_thickness=1, text_scale=0.5)
    verify_annotator  = sv.LineZoneAnnotator(color=sv.Color.YELLOW, thickness=2, text_thickness=1, text_scale=0.5)

    fps_monitor = cv2.TickMeter()

    # ----------------------------------------------------------
    # State tracking
    # ----------------------------------------------------------
    WINDOW_NAME = "Magnesite Conveyor Pipeline"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, DISPLAY_W, DISPLAY_H)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

    frame_count        = 0
    zoom               = ZOOM_FACTOR
    inference_size     = INFERENCE_SIZE
    skip_frames        = SKIP_FRAMES
    last_detections    = None
    last_classifications = []

    # Track whether we did a fresh inference this frame (for LineZone trigger gating)
    fresh_inference_this_frame = False

    try:
        while rclpy.ok():
            # ROS2 Spin
            rclpy.spin_once(ros_node, timeout_sec=0.001)

            frames = pipeline.wait_for_frames()
            # ALIGN DEPTH TO COLOR! Critical for accurate 3D deprojection
            aligned_frames = align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
                
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            full_frame = np.asanyarray(color_frame.get_data())

            # Apply center zoom
            zoomed_frame, crop_rect = center_zoom(full_frame, zoom)

            fps_monitor.start()
            frame_count += 1

            # --------------------------------------------------
            # Inference & Tracking
            # --------------------------------------------------
            run_inference = (frame_count % skip_frames == 0) or last_detections is None

            if run_inference:
                t0 = time.perf_counter()
                results = model(
                    zoomed_frame, verbose=False,
                    conf=CONFIDENCE, imgsz=inference_size,
                    half=USE_HALF
                )[0]
                inference_ms = (time.perf_counter() - t0) * 1000

                # Convert to Supervision detections
                detections = sv.Detections.from_ultralytics(results)

                # Classify Rocks to find Magnesites
                classifications = []
                for i in range(len(detections)):
                    # Extract bounding box/mask and run LAB + Otsu to decide Magnesite vs Rock
                    if detections.mask is not None and i < len(detections.mask) and detections.mask[i] is not None:
                        rock_mask = (detections.mask[i].astype(np.uint8)) * 255
                    else:
                        bx1, by1, bx2, by2 = map(int, detections.xyxy[i])
                        rock_mask = np.zeros(zoomed_frame.shape[:2], dtype=np.uint8)
                        rock_mask[by1:by2, bx1:bx2] = 255

                    is_mag, w_pct, d_pct, ratio, thresh = classify_rock_lab_otsu(zoomed_frame, rock_mask)
                    classifications.append({
                        'is_mag': is_mag, 'w_pct': w_pct, 'd_pct': d_pct
                    })

                # Assign Class IDs: 1 (Magnesite) or 0 (Rock) based on LAB+Otsu result
                class_ids = np.array([1 if c['is_mag'] else 0 for c in classifications], dtype=int)
                detections.class_id = class_ids

                # Let ByteTrack assign/update persistent IDs
                # ByteTrack preserves class_id, so our LAB+Otsu classification is intact.
                tracked_detections = byte_tracker.update_with_detections(detections=detections)

                last_detections = tracked_detections
                fresh_inference_this_frame = True
            else:
                tracked_detections = last_detections
                inference_ms    = 0
                fresh_inference_this_frame = False

            # --------------------------------------------------
            # Line Zone Trigger Logic
            # IMPORTANT: Only trigger on fresh inference frames to avoid double-counting
            # the same rock on repeated stale detections between inference frames.
            # --------------------------------------------------
            n = len(tracked_detections)
            crossed_trigger_in  = np.zeros(n, dtype=bool)
            crossed_trigger_out = np.zeros(n, dtype=bool)
            crossed_verify_in   = np.zeros(n, dtype=bool)
            crossed_verify_out  = np.zeros(n, dtype=bool)

            if fresh_inference_this_frame and tracked_detections is not None and n > 0:
                crossed_trigger_in, crossed_trigger_out = trigger_zone.trigger(detections=tracked_detections)
                crossed_verify_in,  crossed_verify_out  = verify_zone.trigger(detections=tracked_detections)

            # Analyze trigger line
            for i in range(len(tracked_detections)):
                # If the tracker crossed the trigger line this frame (either direction, adjust for belt direction)
                if crossed_trigger_in[i] or crossed_trigger_out[i]:
                    tracker_id = tracked_detections.tracker_id[i]
                    class_id = tracked_detections.class_id[i]
                    
                    if class_id == 1: # It's MAGNESITE
                        # 1. Get pixel center on the ZOOMED frame
                        zx, zy, zw, zh = tracked_detections.xyxy[i]
                        center_x = (zx + zw) / 2.0
                        center_y = (zy + zh) / 2.0
                        
                        # 2. Map pixel center to the ORIGINAL full frame
                        orig_x, orig_y = zoomed_pixel_to_original(
                            center_x, center_y, zoom, crop_rect, DISPLAY_W, DISPLAY_H
                        )

                        # 3. Clamp to valid depth frame bounds (after alignment, depth == color size)
                        df_w = depth_frame.get_width()
                        df_h = depth_frame.get_height()
                        orig_x = max(0, min(orig_x, df_w - 1))
                        orig_y = max(0, min(orig_y, df_h - 1))

                        # 4. Read Depth (using spatial fallback for depth holes)
                        depth = get_valid_depth(depth_frame, orig_x, orig_y, max_radius=5)
                        
                        if depth > 0:
                            # 5. Deproject to 3D Camera Coordinates
                            # rs2_deproject_pixel_to_point expects [x, y] as pixel coordinates
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, [float(orig_x), float(orig_y)], depth
                            )
                            # X is Right, Y is Down, Z is Forward (RealSense optical frame)
                            target_x = float(depth_point[0])
                            target_y = float(depth_point[1])
                            target_z = float(depth_point[2])

                            # 6. Publish to ROS2
                            ros_node.publish_target(target_x, target_y, target_z, tracker_id)
                        else:
                            ros_node.get_logger().error(
                                f"No depth at pixel ({orig_x},{orig_y}) for Magnesite ID: {tracker_id}. "
                                "Check camera alignment and rock distance."
                            )

            # Analyze verification line
            for i in range(len(tracked_detections)):
                if crossed_verify_in[i] or crossed_verify_out[i]:
                    tracker_id = tracked_detections.tracker_id[i]
                    class_id   = tracked_detections.class_id[i]
                    if class_id == 1:
                        # Log/Alert that this Magnesite was NOT removed!
                        alert_msg = f"ALERT! Magnesite ID {tracker_id} reached Verification Line! Robot missed it."
                        ros_node.publish_alert(alert_msg)

            # --------------------------------------------------
            # Visualization
            # --------------------------------------------------
            annotated = zoomed_frame.copy()

            if len(tracked_detections) > 0:
                # If masks exist, render them
                if tracked_detections.mask is not None:
                    annotated = mask_annotator.annotate(scene=annotated, detections=tracked_detections)
                    annotated = polygon_annotator.annotate(scene=annotated, detections=tracked_detections)

                # Draw bounding boxes and Object IDs
                annotated = box_annotator.annotate(scene=annotated, detections=tracked_detections)
                
                labels = [
                    f"ID: {tracker_id} {'MAG' if cls_id == 1 else 'ROCK'}"
                    for tracker_id, cls_id in zip(tracked_detections.tracker_id, tracked_detections.class_id)
                ]
                annotated = label_annotator.annotate(
                    scene=annotated, detections=tracked_detections, labels=labels
                )

            # Render Line Zones
            annotated = trigger_annotator.annotate(annotated, line_counter=trigger_zone)
            annotated = verify_annotator.annotate(annotated, line_counter=verify_zone)

            # Add overlay text for zones (Trigger vs Verify)
            cv2.putText(annotated, "TRIGGER LINE (Robot Action)", (int(TRIGGER_START.x), int(TRIGGER_START.y) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(annotated, "VERIFICATION LINE (Audit)", (int(VERIFY_START.x), int(VERIFY_START.y) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Drawing Mode UI overlays
            if drawing_mode == 'TRIGGER':
                cv2.putText(annotated, "CLICK 2 POINTS FOR TRIGGER LINE", (DISPLAY_W // 2 - 300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            elif drawing_mode == 'VERIFY':
                cv2.putText(annotated, "CLICK 2 POINTS FOR VERIFY LINE", (DISPLAY_W // 2 - 300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            
            if len(current_clicks) == 1:
                cv2.circle(annotated, current_clicks[0], 5, (255, 255, 255), -1)

            # Draw zoom indicator
            cv2.putText(annotated, f"ZOOM {zoom:.1f}x",
                        (DISPLAY_W // 2 - 50, DISPLAY_H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

            fps_monitor.stop()
            current_fps = fps_monitor.getFPS()

            # Info panel
            panel_h, panel_w = 130, 420
            roi_p = annotated[5:panel_h, 5:panel_w]
            annotated[5:panel_h, 5:panel_w] = (roi_p * 0.4).astype(np.uint8)

            cv2.putText(annotated, f"FPS: {current_fps:.1f}  |  Inference: {inference_ms:.0f}ms",
                        (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(annotated, f"Targets Triggered: {trigger_zone.in_count + trigger_zone.out_count}",
                        (15, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(annotated, f"Pipeline: LAB+Otsu > {MAGNESITE_THRESHOLD}%",
                        (15, 69), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
            cv2.putText(annotated, "Press 'h' for controls | 'q' to quit",
                        (15, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (150, 150, 150), 1)
            cv2.putText(annotated, "Press 't' draw trigger | 'v' draw verify",
                        (15, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

            cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('t'):
                drawing_mode = 'TRIGGER'
                current_clicks = []
                print("Click 2 points to draw the TRIGGER line...")
            elif key == ord('v'):
                drawing_mode = 'VERIFY'
                current_clicks = []
                print("Click 2 points to draw the VERIFY line...")
            elif key == ord('s'):
                ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
                path = f"magnesite_conveyor_{ts}.png"
                cv2.imwrite(path, annotated)
                print(f"Screenshot saved: {path}")
            elif key == ord('+') or key == ord('='):
                zoom = min(zoom + 0.25, 8.0)
            elif key == ord('-') or key == ord('_'):
                zoom = max(zoom - 0.25, 1.0)
            elif key == ord('i'):
                inference_size = min(inference_size + 128, 1280)
            elif key == ord('d'):
                inference_size = max(inference_size - 128, 320)
            elif key == ord('h'):
                print("Controls: +/- Zoom, i/d Inference Size, s Screenshot, q Quit")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        print("\nStopping...")
        pipeline.stop()
        cv2.destroyAllWindows()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Done.")

if __name__ == "__main__":
    main()
