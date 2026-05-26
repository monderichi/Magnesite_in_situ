#!/usr/bin/env python3
"""
Robot Self-Filter - Simplified version

ONLY removes points that belong to the robot arm.
Does NOT remove ground plane or crop workspace.
Preserves all other points for collision detection.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener
import tf_transformations

import numpy as np
from typing import List, Tuple


class RobotSelfFilter(Node):
    """
    Simple self-filter: only removes robot arm points.
    Everything else passes through unchanged.
    """

    def __init__(self):
        super().__init__('robot_self_filter')

        # Parameters - ONLY robot filtering
        self.declare_parameter('input_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/camera/depth/color/points_self_filtered')
        self.declare_parameter('robot_frame', 'base_link')
        
        # Robot self-filter parameters
        self.declare_parameter('robot_filter_padding', 0.03)  # 3cm padding around links
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.robot_filter_padding = self.get_parameter('robot_filter_padding').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/self_filter/markers', 10)
        
        # Subscriber
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            10
        )

        # Statistics
        self.frame_count = 0
        self.get_logger().info(
            f'RobotSelfFilter initialized:\n'
            f'  Input: {self.input_topic}\n'
            f'  Output: {self.output_topic}\n'
            f'  Robot padding: {self.robot_filter_padding}m\n'
            f'  ONLY removing robot arm points - all other points preserved'
        )

    def get_robot_capsules(self) -> List[Tuple[str, np.ndarray, np.ndarray, float]]:
        """Get robot link capsules (line segments with radius) for filtering.
        
        Returns list of (name, point_a, point_b, radius) tuples.
        Capsules connect consecutive link origins, covering the full arm segments.
        """
        # Define the kinematic chain and radius for each segment
        # Each entry: (link_name, radius)
        link_chain = [
            ('base_link', 0.09),
            ('shoulder_link', 0.08),
            ('upper_arm_link', 0.07),
            ('forearm_link', 0.06),
            ('wrist_1_link', 0.05),
            ('wrist_2_link', 0.05),
            ('wrist_3_link', 0.04),
        ]
        
        # Look up all link positions
        link_positions = {}
        for link_name, _ in link_chain:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    link_name,
                    rclpy.time.Time()
                )
                link_positions[link_name] = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
            except Exception as e:
                self.get_logger().debug(f'TF failed for {link_name}: {e}')
        
        capsules = []
        
        # Build capsules between consecutive links in the chain
        for i in range(len(link_chain) - 1):
            name_a, _ = link_chain[i]
            name_b, radius_b = link_chain[i + 1]
            
            if name_a in link_positions and name_b in link_positions:
                pos_a = link_positions[name_a]
                pos_b = link_positions[name_b]
                radius = max(radius_b, link_chain[i][1]) + self.robot_filter_padding
                capsules.append((f'{name_a}->{name_b}', pos_a, pos_b, radius))
        
        # Also add a sphere at base_link (the stationary base)
        if 'base_link' in link_positions:
            pos = link_positions['base_link']
            capsules.append(('base_link', pos, pos, 0.09 + self.robot_filter_padding))
        
        return capsules

    @staticmethod
    def _point_to_segment_distance(points: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray) -> np.ndarray:
        """Vectorized point-to-line-segment distance for all points.
        
        Returns the minimum distance from each point to the line segment AB.
        """
        ab = seg_b - seg_a
        ab_len_sq = np.dot(ab, ab)
        
        if ab_len_sq < 1e-10:
            # Degenerate segment (A == B), just distance to point A
            return np.linalg.norm(points[:, :3] - seg_a, axis=1)
        
        # Project each point onto the line AB, clamped to [0, 1]
        ap = points[:, :3] - seg_a
        t = np.clip(np.dot(ap, ab) / ab_len_sq, 0.0, 1.0)
        
        # Closest point on segment for each input point
        closest = seg_a + np.outer(t, ab)
        
        return np.linalg.norm(points[:, :3] - closest, axis=1)

    def filter_robot_points(self, points: np.ndarray, robot_capsules) -> np.ndarray:
        """Remove points inside robot collision capsules."""
        if not robot_capsules:
            return points
        
        # Start with all points kept
        keep_mask = np.ones(len(points), dtype=bool)
        
        # For each capsule, mark points inside it as filtered
        for name, seg_a, seg_b, radius in robot_capsules:
            distances = self._point_to_segment_distance(points, seg_a, seg_b)
            # Keep points that are OUTSIDE the capsule
            keep_mask &= (distances > radius)
        
        return points[keep_mask]

    def publish_markers(self, robot_capsules):
        """Publish markers showing capsule collision volumes."""
        marker_array = MarkerArray()
        
        for i, (name, seg_a, seg_b, radius) in enumerate(robot_capsules):
            # Sphere at each endpoint
            for j, pos in enumerate([seg_a, seg_b]):
                marker = Marker()
                marker.header.frame_id = self.robot_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'robot_self_filter'
                marker.id = i * 10 + j
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(pos[0])
                marker.pose.position.y = float(pos[1])
                marker.pose.position.z = float(pos[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = float(radius * 2)
                marker.scale.y = float(radius * 2)
                marker.scale.z = float(radius * 2)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.15
                marker.lifetime.sec = 1
                marker_array.markers.append(marker)
        
        self.pub_markers.publish(marker_array)

    def cloud_callback(self, msg: PointCloud2):
        """Process point cloud - only remove robot points."""
        try:
            # Convert to numpy
            points_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
            
            if len(points_list) == 0:
                return
            
            points = np.array(points_list)
            original_count = len(points)

            # Transform points to robot frame for filtering
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().debug(f'TF lookup failed: {e}')
                # If TF fails, pass through original cloud
                self.pub_filtered.publish(msg)
                return

            # Get transform matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z
            
            # Transform points to robot frame
            points_homogeneous = np.hstack([points[:, :3], np.ones((len(points), 1))])
            points_robot_frame = (T @ points_homogeneous.T).T[:, :3]
            
            # Get current robot capsules
            robot_capsules = self.get_robot_capsules()
            
            # Build keep mask using capsule-based filtering
            keep_mask = np.ones(original_count, dtype=bool)
            for name, seg_a, seg_b, radius in robot_capsules:
                distances = self._point_to_segment_distance(points_robot_frame, seg_a, seg_b)
                keep_mask &= (distances > radius)
            
            # Publish debug markers
            self.publish_markers(robot_capsules)
            
            # Apply mask to get filtered points in camera frame
            points_camera_frame = points[keep_mask, :3]
            
            # Recombine with RGB using the mask directly
            if points.shape[1] > 3:
                points_camera_frame = np.hstack([points_camera_frame, points[keep_mask, 3:]])

            filtered_count = len(points_camera_frame)
            self.frame_count += 1

            # Build output message
            if filtered_count > 0:
                # Determine fields
                has_rgb = points_camera_frame.shape[1] > 3
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                if has_rgb:
                    fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
                    point_step = 16
                else:
                    point_step = 12

                out_msg = PointCloud2()
                out_msg.header = msg.header  # Keep original frame (camera)
                out_msg.height = 1
                out_msg.width = filtered_count
                out_msg.fields = fields
                out_msg.is_bigendian = False
                out_msg.point_step = point_step
                out_msg.row_step = point_step * filtered_count
                out_msg.is_dense = False
                out_msg.data = points_camera_frame.astype(np.float32).tobytes()
                
                self.pub_filtered.publish(out_msg)
                
                # Log stats occasionally
                if self.frame_count <= 3 or self.frame_count % 30 == 0:
                    removed = original_count - filtered_count
                    self.get_logger().info(
                        f'Frame {self.frame_count}: {original_count} -> {filtered_count} '
                        f'(removed {removed} robot points, {100*removed/original_count:.1f}%)'
                    )
            else:
                # All points were robot - publish empty
                self.get_logger().warn('All points were robot points! Publishing empty cloud.')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = RobotSelfFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
