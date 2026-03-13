#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class EndEffectorVelocity(Node):
    def __init__(self):
        super().__init__('ee_velocity_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.prev_pos = None
        self.prev_time = None
        self.timer = self.create_timer(0.1, self.compute_velocity)  # 10 Hz

    def compute_velocity(self):
        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',   # base frame
                'tool0',       # end-effector frame
                rclpy.time.Time()
            )
            t = transform.transform.translation
            curr_time = self.get_clock().now().nanoseconds / 1e9

            if self.prev_pos is not None:
                dx = t.x - self.prev_pos[0]
                dy = t.y - self.prev_pos[1]
                dz = t.z - self.prev_pos[2]
                dt = curr_time - self.prev_time
                if dt > 0:
                    speed = (dx**2 + dy**2 + dz**2)**0.5 / dt
                    self.get_logger().info(f"Speed: {speed:.4f} m/s")

            self.prev_pos = (t.x, t.y, t.z)
            self.prev_time = curr_time

        except Exception as e:
            self.get_logger().warn(f"Transform not ready: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()