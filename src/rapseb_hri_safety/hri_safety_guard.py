#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RAPSEB HRI Safety Guard
- Subscribes to ros4hri tracked IDs
- Looks up /humans/<id>/body in TF and computes distance to robot base
- Enforces ISO-style zones:
    Z1  -> STOP (pause controller)
    Z2  -> REDUCED (try to lower speed slider, else just publish mode)
    Z3  -> NORMAL (resume controller if stopped)
- Publishes /rapseb/robot_mode = NORMAL | REDUCED | STOPPED
"""

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController

# If ros4hri is present:
try:
    from hri_msgs.msg import Ids  # list of tracked human IDs
    HRI_AVAILABLE = True
except Exception:
    HRI_AVAILABLE = False


class HRISafetyGuard(Node):
    def __init__(self):
        super().__init__('hri_safety_guard')

        # --- Parameters ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tracked_topic', '/humans/tracked_ids')  # ros4hri default
        self.declare_parameter('stop_distance_m', 1.0)   # Z1 threshold
        self.declare_parameter('warn_distance_m', 1.5)   # Z2 threshold
        self.declare_parameter('controller_manager_ns', '/controller_manager')
        self.declare_parameter('trajectory_controller', 'scaled_joint_trajectory_controller')
        self.declare_parameter('monitor_rate_hz', 20.0)
        self.declare_parameter('speed_slider_service', '')  # e.g. /dashboard_client/set_speed_slider_fraction
        self.declare_parameter('reduced_speed_percent', 20) # requested reduced speed if service is available
        self.declare_parameter('tf_timeout_sec', 0.08)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tracked_topic = self.get_parameter('tracked_topic').get_parameter_value().string_value
        self.stop_distance = self.get_parameter('stop_distance_m').get_parameter_value().double_value
        self.warn_distance = self.get_parameter('warn_distance_m').get_parameter_value().double_value
        self.ctrl_mgr_ns = self.get_parameter('controller_manager_ns').get_parameter_value().string_value
        self.traj_ctrl = self.get_parameter('trajectory_controller').get_parameter_value().string_value
        self.monitor_rate = self.get_parameter('monitor_rate_hz').get_parameter_value().double_value
        self.speed_srv_name = self.get_parameter('speed_slider_service').get_parameter_value().string_value
        self.reduced_speed = int(self.get_parameter('reduced_speed_percent').get_parameter_value().integer_value)
        self.tf_timeout = self.get_parameter('tf_timeout_sec').get_parameter_value().double_value

        if self.warn_distance <= self.stop_distance:
            self.get_logger().warn('warn_distance_m should be > stop_distance_m; adjusting automatically')
            self.warn_distance = self.stop_distance + 0.2

        # --- TF Buffer & Listener ---
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ros4hri tracked IDs subscription (fallback to manual list if not available) ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tracked_ids: List[str] = []
        if HRI_AVAILABLE:
            self.sub_tracked = self.create_subscription(Ids, self.tracked_topic, self._tracked_cb, qos)
            self.get_logger().info(f'Listening for tracked IDs on {self.tracked_topic}')
        else:
            self.get_logger().warn('hri_msgs not available. Provide tracked IDs via parameter or implement a custom subscriber.')

        # --- robot mode publisher ---
        self.mode_pub = self.create_publisher(String, '/rapseb/robot_mode', 10)
        self.current_mode = 'NORMAL'
        self.last_mode_change = time.time()

        # --- Controller manager client ---
        self.switch_cli = self.create_client(SwitchController, f'{self.ctrl_mgr_ns}/switch_controller')

        # Optional speed slider client (if your UR driver exposes one)
        self.speed_cli = None
        if self.speed_srv_name:
            try:
                # Lazy import to avoid hard dependency if service is not used
                from ur_dashboard_msgs.srv import SetSpeedSliderFraction
                self.SpeedSrvType = SetSpeedSliderFraction
                self.speed_cli = self.create_client(self.SpeedSrvType, self.speed_srv_name)
                self.get_logger().info(f'Using speed slider service at {self.speed_srv_name}')
            except Exception:
                self.get_logger().warn('UR speed slider service type not available, will rely on controller switching only')
                self.speed_cli = None

        # --- Timer for monitoring loop ---
        self.timer = self.create_timer(1.0 / self.monitor_rate, self._monitor_step)

    # --- Callbacks and helpers ---

    def _tracked_cb(self, msg: 'Ids'):
        # hri_msgs/Ids has field "ids"
        self.tracked_ids = list(msg.ids)

    def _publish_mode(self, mode: str):
        if mode != self.current_mode:
            self.current_mode = mode
            self.last_mode_change = time.time()
            self.mode_pub.publish(String(data=mode))
            self.get_logger().info(f'Robot mode -> {mode}')

    def _nearest_human_distance(self) -> Optional[float]:
        # Try all tracked IDs; compute distance to each /humans/<id>/body frame
        nearest = None
        for hid in self.tracked_ids:
            body_frame = f'humans/{hid}/body'
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, body_frame, rclpy.time.Time(),
                                                     timeout=Duration(seconds=self.tf_timeout))
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z
                d = math.sqrt(x*x + y*y + z*z)
                if nearest is None or d < nearest:
                    nearest = d
            except TransformException:
                # Could not resolve this id right now; continue
                continue
        return nearest

    # --- Backends ---

    async def _set_speed_slider(self, percent: int) -> bool:
        if not self.speed_cli:
            return False
        if not self.speed_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Speed slider service not available')
            return False
        try:
            req = self.SpeedSrvType.Request()
            # Different UR variants use 0-100 integer or 0.0-1.0 float fraction.
            # We normalize here to integer percent if the field exists.
            if hasattr(req, 'speed_slider_fraction'):
                # Heuristic: if type is float, use 0.0-1.0; if int, use 0-100
                if isinstance(getattr(req, 'speed_slider_fraction'), float):
                    req.speed_slider_fraction = max(0.0, min(1.0, percent / 100.0))
                else:
                    req.speed_slider_fraction = int(max(0, min(100, percent)))
            elif hasattr(req, 'speed_slider'):
                req.speed_slider = int(max(0, min(100, percent)))
            else:
                self.get_logger().warn('Unknown speed slider request field; skipping')
                return False

            fut = self.speed_cli.call_async(req)
            await fut
            return True
        except Exception as e:
            self.get_logger().warn(f'Speed slider call failed: {e}')
            return False

    async def _switch_controller(self, start: List[str], stop: List[str]) -> bool:
        if not self.switch_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Controller manager service not available')
            return False
        try:
            req = SwitchController.Request()
            req.start_controllers = start
            req.stop_controllers = stop
            # Strictness 1 BEST_EFFORT, 2 STRICT in Humble
            req.strictness = 2
            req.start_asap = True
            req.timeout = 2.0
            fut = self.switch_cli.call_async(req)
            await fut
            return fut.result() is not None
        except Exception as e:
            self.get_logger().warn(f'Controller switch failed: {e}')
            return False

    # --- Main monitor loop ---

    def _monitor_step(self):
        # If no tracked humans data, fail-safe to STOP
        d = self._nearest_human_distance()
        if d is None:
            # conservative default if HRI stream is lost for more than a brief moment
            self._publish_mode('STOPPED')
            self._ensure_stopped()
            return

        # Decide zone and action
        if d <= self.stop_distance:
            self._publish_mode('STOPPED')
            self._ensure_stopped()
        elif d <= self.warn_distance:
            self._publish_mode('REDUCED')
            self._ensure_reduced()
        else:
            self._publish_mode('NORMAL')
            self._ensure_running()

    # --- Action drivers ---

    def _ensure_stopped(self):
        # Switch off trajectory controller if not already stopped
        # Also try to set speed slider to 0 if available
        rclpy.task.Future
        self.create_task(self._set_speed_slider(0))
        self.create_task(self._switch_controller(start=[], stop=[self.traj_ctrl]))

    def _ensure_reduced(self):
        # Keep controller running but try to reduce speed
        # If slider is unavailable, just publish mode and keep controller active
        if self.speed_cli:
            self.create_task(self._set_speed_slider(self.reduced_speed))

    def _ensure_running(self):
        # Restore normal speed and ensure controller is active
        if self.speed_cli:
            self.create_task(self._set_speed_slider(100))
        self.create_task(self._switch_controller(start=[self.traj_ctrl], stop=[]))

    # Helper to schedule coroutines without asyncio boilerplate
    def create_task(self, coro):
        # rclpy has no built-in loop; we can spin coroutines using timers
        # Minimal adapter for fire-and-forget calls
        def _done_cb(fut):
            exc = fut.exception() if hasattr(fut, 'exception') else None
            if exc:
                self.get_logger().warn(f'Async op error: {exc}')
        fut = rclpy.task.Future()  # placeholder for type hints
        try:
            fut = coro  # in Foxy/Humble, call_async returns a Future
            fut.add_done_callback(_done_cb)
        except Exception:
            pass
        return fut


def main(args=None):
    rclpy.init(args=args)
    node = HRISafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
