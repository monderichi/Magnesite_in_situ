#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from datetime import datetime
from pathlib import Path
import math
import csv
import time


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        # Logging directory
        log_dir = Path('/ros2_ws/src/spraying_pathways/robot_logs')
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir = log_dir
        self.get_logger().info(f'Logging trajectory data to: {self.log_dir}')

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}

        # EE speed tracking
        self.last_ee_position = None
        self.last_ee_time = None

        # End-effector frame
        self.end_effector_frame = None
        self.detect_end_effector_frame()

        # Logging state
        self.csv_file = None
        self.csv_writer = None
        self.logging_started = False

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Timer
        self.create_timer(0.1, self.log_data)

    def detect_end_effector_frame(self):
        self.get_logger().info('Detecting end-effector frame from TF...')
        time.sleep(2.0)
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            candidate_frames = []

            for line in frames_yaml.splitlines():
                if 'child_frame_id' in line:
                    frame = line.split(':')[-1].strip()
                    if frame not in ['base_link', 'world', 'odom', 'map'] and not frame.startswith('camera'):
                        candidate_frames.append(frame)

            if candidate_frames:
                self.end_effector_frame = max(candidate_frames, key=len)
                self.get_logger().info(f'Using end-effector frame: {self.end_effector_frame}')
            else:
                raise RuntimeError('No suitable end-effector frame found.')

        except Exception as e:
            self.get_logger().warn(f'EE frame detection failed, using default "tool0": {e}')
            self.end_effector_frame = 'tool0'

    def joint_state_cb(self, msg):
        self.joint_names = msg.name
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0

        if not self.logging_started:
            self.start_logging()

    def start_logging(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = self.log_dir / f'trajectory_log_{timestamp}.csv'
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        joint_headers = []
        for name in self.joint_names:
            joint_headers.extend([f'{name}_pos', f'{name}_vel'])

        header = ['time'] + joint_headers + ['ee_x', 'ee_y', 'ee_z', 'ee_roll', 'ee_pitch', 'ee_yaw', 'ee_speed']
        self.csv_writer.writerow(header)

        self.logging_started = True
        self.get_logger().info(f'Started logging to {filename}')

    def stop_logging(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_writer = None
            self.logging_started = False
            self.get_logger().info('Stopped logging and saved CSV.')

    def log_data(self):
        if not self.logging_started or not self.csv_writer:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        joint_values = []
        for name in self.joint_names:
            pos = self.joint_positions.get(name, 0.0)
            vel = self.joint_velocities.get(name, 0.0)
            joint_values.extend([pos, vel])

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.end_effector_frame,
                rclpy.time.Time()
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])

            current_pos = (t.x, t.y, t.z)
            current_time = now

            # Compute speed
            if self.last_ee_position is not None and self.last_ee_time is not None:
                dx = current_pos[0] - self.last_ee_position[0]
                dy = current_pos[1] - self.last_ee_position[1]
                dz = current_pos[2] - self.last_ee_position[2]
                dt = current_time - self.last_ee_time
                ee_speed = math.sqrt(dx**2 + dy**2 + dz**2) / dt if dt > 0 else 0.0
                
            else:
                ee_speed = 0.0

            self.last_ee_position = current_pos
            self.last_ee_time = current_time

            ee_values = [t.x, t.y, t.z, roll, pitch, yaw, ee_speed]

        except Exception as e:
            self.get_logger().warn_once(f'EE transform not available: {e}')
            ee_values = [0.0] * 7

        self.csv_writer.writerow([now] + joint_values + ee_values)

    def destroy_node(self):
        self.stop_logging()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level('trajectory_logger', rclpy.logging.LoggingSeverity.INFO)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
