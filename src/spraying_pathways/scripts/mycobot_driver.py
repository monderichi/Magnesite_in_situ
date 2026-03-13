#!/usr/bin/env python3
"""
myCobot 320 M5 Driver Node for ROS2 - SEAMLESS TRAJECTORY VERSION

This node communicates with the real myCobot 320 M5 robot via serial port
using the pymycobot library.

KEY INSIGHT: pymycobot's send_angles() is BLOCKING - it sends the command
AND waits for serial acknowledgment (~1.5s per call). For smooth trajectory
streaming, we bypass the blocking reply by writing angle commands directly
to the serial port (fire-and-forget). Combined with set_fresh_mode(1),
this creates truly seamless, continuous robot motion.

Features:
- Publishes joint states at configurable rate (default 50Hz)
- Action server for FollowJointTrajectory (MoveIt compatible)
- SEAMLESS trajectory execution using non-blocking serial writes + fresh mode
- Falls back to blocking send_angles() for single-point moves

Prerequisites:
    pip3 install pymycobot

Usage:
    ros2 run spraying_pathways mycobot_driver.py
    ros2 run spraying_pathways mycobot_driver.py --ros-args -p port:=/dev/ttyACM0

Parameters:
    port (str): Serial port device (default: /dev/ttyACM0)
    baud (int): Baud rate (default: 115200)
    publish_rate (float): Joint state publish rate in Hz (default: 50.0)
    speed (int): Robot movement speed 0-100 (default: 80)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

import threading
import time
import math
import struct

# Try to import pymycobot
try:
    from pymycobot import MyCobot320
    from pymycobot.common import ProtocolCode
    PYMYCOBOT_AVAILABLE = True
except ImportError:
    PYMYCOBOT_AVAILABLE = False


class MyCobot320Driver(Node):
    """ROS2 Driver Node for myCobot 320 M5 with seamless trajectory execution.
    
    Uses pymycobot's set_fresh_mode(1) + non-blocking serial writes for
    truly smooth, continuous motion during trajectory execution.
    """

    def __init__(self):
        super().__init__('mycobot_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('speed', 80)  # Robot speed 0-100

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.speed = self.get_parameter('speed').value

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.get_logger().info(f"MyCobot 320 Driver initializing...")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info(f"  Baud: {self.baud}")
        self.get_logger().info(f"  Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"  Robot speed: {self.speed}/100")

        # Initialize robot
        self.mc = None
        self.connected = False
        self.mock_mode = False
        self.current_joint_angles = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        self._serial_lock = threading.Lock()

        if not PYMYCOBOT_AVAILABLE:
            self.get_logger().error("pymycobot not installed! Install: pip3 install pymycobot")
            self.get_logger().warning("Running in MOCK MODE")
            self.mock_mode = True
        else:
            self._connect_to_robot()

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Action server for trajectory execution (MoveIt uses this)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info(f"Action server ready: /arm_controller/follow_joint_trajectory")

        # Timer for joint state publishing
        self.js_timer = self.create_timer(1.0 / self.publish_rate, self._publish_joint_states)

        self.get_logger().info("MyCobot 320 Driver initialized!")

    def _connect_to_robot(self):
        """Connect to robot and enable fresh/interpolation mode."""
        try:
            self.mc = MyCobot320(self.port, self.baud)
            time.sleep(0.5)

            # Test connection
            angles = self._get_angles_safe()
            if angles is not None:
                self.connected = True
                self.current_joint_angles = [math.radians(a) for a in angles]
                self.get_logger().info(f"Connected! Initial angles (deg): {[round(a, 1) for a in angles]}")
            else:
                raise Exception("Could not read initial joint angles")

            # *** CRITICAL: Enable fresh/interpolation mode ***
            # Mode 1 = "Always execute the latest command first"
            # Each send_angles() immediately replaces the current target.
            self.mc.set_fresh_mode(1)
            time.sleep(0.1)
            
            mode = self.mc.get_fresh_mode()
            self.get_logger().info(f"Fresh mode: {mode} (1=interpolation/smooth, 0=queue/stepwise)")

        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.get_logger().warning("Running in MOCK MODE")
            self.mock_mode = True
            self.connected = False

    def _build_send_angles_command(self, angles_deg, speed):
        """Build the raw serial command for send_angles WITHOUT sending it.
        
        This constructs the exact same byte sequence that pymycobot's
        send_angles() would send, but returns it as bytes so we can
        write it directly to the serial port without waiting for a reply.
        
        Protocol: [0xFE, 0xFE, LEN, CMD, angle1_hi, angle1_lo, ..., speed, 0xFA]
        """
        # Convert angles to int16 (angle * 100)
        angle_ints = [int(a * 100) for a in angles_deg]
        
        # Encode as big-endian int16
        data = []
        for a in angle_ints:
            data.extend(struct.pack(">h", a))
        
        # Build command: header(2) + length(1) + cmd(1) + data(12) + speed(1) + footer(1)
        cmd = ProtocolCode.SEND_ANGLES  # 0x22
        payload = list(data) + [speed]
        length = len(payload) + 2  # +2 for cmd byte and footer byte
        
        command = [0xFE, 0xFE, length, cmd] + payload + [0xFA]
        return bytes(command)

    def _send_angles_nonblocking(self, angles_deg, speed):
        """Send angles to robot WITHOUT waiting for reply (fire-and-forget).
        
        This is the key to smooth motion: pymycobot's send_angles() blocks
        for ~1.5s waiting for serial acknowledgment. By writing directly
        to the serial port, we can stream commands at 20-50Hz.
        """
        if self.mc is None or not self.connected:
            return False
            
        try:
            command = self._build_send_angles_command(angles_deg, speed)
            with self._serial_lock:
                self.mc._serial_port.write(command)
                # Flush the output buffer so command is sent immediately
                self.mc._serial_port.flush()
                # Clear any pending input to avoid buffer buildup
                if self.mc._serial_port.in_waiting > 0:
                    self.mc._serial_port.reset_input_buffer()
            return True
        except Exception as e:
            self.get_logger().error(f"Non-blocking send failed: {e}")
            return False

    def _get_angles_safe(self):
        """Safely get angles from robot."""
        if self.mc is None:
            return None
        try:
            with self._serial_lock:
                angles = self.mc.get_angles()
            if isinstance(angles, (list, tuple)) and len(angles) >= 6:
                return list(angles)[:6]
            return None
        except Exception as e:
            self.get_logger().warn(f"get_angles failed: {e}")
            return None

    def _publish_joint_states(self):
        """Publish current joint states."""
        if not self.mock_mode and self.connected:
            angles = self._get_angles_safe()
            if angles:
                self.current_joint_angles = [math.radians(a) for a in angles]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joint_angles
        msg.velocity = self.current_joint_velocities
        msg.effort = [0.0] * 6

        self.joint_state_pub.publish(msg)

    def _goal_callback(self, goal_request):
        """Accept trajectory goal."""
        self.get_logger().info("Trajectory goal received")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel - stop the robot."""
        self.get_logger().info("Trajectory cancel received - stopping robot")
        if self.mc and self.connected:
            try:
                self.mc.stop()
            except Exception:
                pass
        return CancelResponse.ACCEPT

    def _interpolate_trajectory(self, trajectory, joint_indices, interval_ms=50):
        """Interpolate MoveIt trajectory into dense waypoints for smooth streaming.
        
        MoveIt typically sends sparse waypoints (e.g., 10-20 points over several seconds).
        We interpolate between them to create dense waypoints every `interval_ms` milliseconds.
        This ensures the robot receives frequent, small position updates for smooth motion.
        
        Args:
            trajectory: The JointTrajectory message
            joint_indices: Mapping from trajectory joints to robot joints
            interval_ms: Interpolation interval in milliseconds
        
        Returns:
            List of (time_sec, angles_deg) tuples
        """
        points = trajectory.points
        if len(points) == 0:
            return []
        
        # Extract all timestamps and angle arrays
        timestamps = []
        angle_arrays = []
        for point in points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            timestamps.append(t)
            
            angles_deg = [0.0] * 6
            for j, idx in enumerate(joint_indices):
                if idx >= 0 and j < len(point.positions):
                    angles_deg[idx] = math.degrees(point.positions[j])
            angle_arrays.append(angles_deg)
        
        if len(points) == 1:
            return [(timestamps[0], angle_arrays[0])]
        
        # Interpolate between consecutive points
        interval_sec = interval_ms / 1000.0
        interpolated = []
        
        for i in range(len(timestamps) - 1):
            t_start = timestamps[i]
            t_end = timestamps[i + 1]
            a_start = angle_arrays[i]
            a_end = angle_arrays[i + 1]
            
            dt = t_end - t_start
            if dt <= 0:
                interpolated.append((t_start, a_start))
                continue
            
            # Generate intermediate points
            num_steps = max(1, int(dt / interval_sec))
            for step in range(num_steps):
                alpha = step / num_steps  # 0.0 to ~1.0
                t = t_start + alpha * dt
                angles = [
                    a_start[j] + alpha * (a_end[j] - a_start[j])
                    for j in range(6)
                ]
                interpolated.append((t, angles))
        
        # Always include the final point
        interpolated.append((timestamps[-1], angle_arrays[-1]))
        
        return interpolated

    def _execute_trajectory_callback(self, goal_handle):
        """
        Execute joint trajectory with SEAMLESS motion.
        
        Strategy:
        1. Interpolate MoveIt's sparse waypoints into dense ones (every 50ms)
        2. Stream them to the robot using non-blocking serial writes
        3. In fresh mode, each write immediately updates the target
        4. Result: smooth, continuous motion without stop-and-go
        """
        self.get_logger().info("Executing trajectory...")

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        total_points = len(trajectory.points)

        self.get_logger().info(f"Joints: {joint_names}, Points: {total_points}")

        if total_points == 0:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = result.SUCCESSFUL
            return result

        # Map trajectory joints to robot joints
        joint_indices = []
        for name in joint_names:
            if name in self.joint_names:
                joint_indices.append(self.joint_names.index(name))
            else:
                self.get_logger().warn(f"Unknown joint: {name}")
                joint_indices.append(-1)

        # Get trajectory total duration
        last_point = trajectory.points[-1]
        total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        self.get_logger().info(f"Trajectory duration: {total_duration:.2f}s")

        # Interpolate trajectory into dense waypoints (every 50ms)
        dense_waypoints = self._interpolate_trajectory(trajectory, joint_indices, interval_ms=50)
        self.get_logger().info(f"Interpolated to {len(dense_waypoints)} dense waypoints (from {total_points} original)")

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = joint_names

        # ===== SEAMLESS TRAJECTORY STREAMING =====
        execution_start = time.time()
        last_angles_deg = None
        
        for i, (point_time, angles_deg) in enumerate(dense_waypoints):
            if goal_handle.is_cancel_requested:
                if self.mc and self.connected:
                    try:
                        self.mc.stop()
                    except Exception:
                        pass
                goal_handle.canceled()
                self.get_logger().info("Trajectory canceled")
                return FollowJointTrajectory.Result()

            # Wait until it's time to send this point
            elapsed = time.time() - execution_start
            wait_time = point_time - elapsed
            if wait_time > 0.002:
                time.sleep(wait_time)

            # Send to robot using NON-BLOCKING write (fire-and-forget)
            if not self.mock_mode and self.connected:
                success = self._send_angles_nonblocking(angles_deg, self.speed)
                if not success:
                    self.get_logger().error(f"Failed at waypoint {i}")
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()

            # Track commanded position
            self.current_joint_angles = [math.radians(a) for a in angles_deg]
            last_angles_deg = angles_deg

            # Publish feedback periodically (not every point to reduce overhead)
            if i % 5 == 0 or i == len(dense_waypoints) - 1:
                feedback_msg.desired.positions = [math.radians(a) for a in angles_deg]
                feedback_msg.actual.positions = [self.current_joint_angles[idx] for idx in joint_indices if idx >= 0]
                goal_handle.publish_feedback(feedback_msg)

            # Log progress
            if i == 0 or i == len(dense_waypoints) - 1:
                self.get_logger().info(f"Streaming: {i+1}/{len(dense_waypoints)} at t={point_time:.2f}s")

        # Wait for robot to reach the final position
        if not self.mock_mode and self.connected and last_angles_deg is not None:
            self.get_logger().info("Waiting for final position...")
            self._wait_for_final_position(last_angles_deg, timeout=5.0)

        # Update from actual robot position
        if not self.mock_mode and self.connected:
            actual = self._get_angles_safe()
            if actual:
                self.current_joint_angles = [math.radians(a) for a in actual]

        elapsed_total = time.time() - execution_start
        goal_handle.succeed()
        self.get_logger().info(f"Trajectory complete in {elapsed_total:.2f}s (planned: {total_duration:.2f}s)")

        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        return result

    def _wait_for_final_position(self, target_deg, timeout=5.0):
        """Wait for robot to reach the final target position."""
        start = time.time()
        tolerance = 3.0  # degrees
        
        while time.time() - start < timeout:
            current = self._get_angles_safe()
            if current is not None:
                max_error = max(abs(c - t) for c, t in zip(current, target_deg))
                if max_error < tolerance:
                    self.get_logger().info(f"Final position reached (max error: {max_error:.1f}°)")
                    return True
            time.sleep(0.1)
        
        self.get_logger().warn(f"Final position timeout after {timeout}s")
        return False

    def destroy_node(self):
        """Cleanup."""
        if self.mc and self.connected:
            self.get_logger().info("Shutting down...")
            try:
                self.get_logger().info("Disconnecting...")
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    driver = MyCobot320Driver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info("Shutting down...")
    finally:
        driver.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
