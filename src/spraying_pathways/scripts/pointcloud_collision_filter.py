#!/usr/bin/env python3
"""
Point Cloud Collision Filter

Filters the RealSense point cloud to remove:
1. Ground plane (table surface)
2. Robot self-seen points
3. Outliers and noise
4. Points outside workspace

Publishes filtered cloud for MoveIt collision avoidance.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs import do_transform_cloud
import tf_transformations

import numpy as np
from typing import List, Tuple, Optional


class PointCloudCollisionFilter(Node):
    """
    Filters point cloud for MoveIt collision avoidance.
    Removes robot self-seen points, ground plane, and noise.
    """

    def __init__(self):
        super().__init__('pointcloud_collision_filter')

        # Parameters
        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/color/points_filtered')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        
        # Ground removal
        self.declare_parameter('ground_z_threshold', 0.03)  # Remove points below 3cm in base_link
        
        # Robot self-filter (simple sphere-based filtering)
        self.declare_parameter('robot_filter_enabled', True)
        self.declare_parameter('robot_filter_padding', 0.05)  # 5cm padding
        
        # Workspace bounds (in base_link frame)
        self.declare_parameter('workspace_min_x', -0.2)
        self.declare_parameter('workspace_max_x', 1.0)
        self.declare_parameter('workspace_min_y', -0.6)
        self.declare_parameter('workspace_max_y', 0.6)
        self.declare_parameter('workspace_min_z', 0.02)
        self.declare_parameter('workspace_max_z', 1.0)
        
        # Outlier removal
        self.declare_parameter('outlier_removal_enabled', False)  # CPU intensive
        self.declare_parameter('outlier_mean_k', 20)
        self.declare_parameter('outlier_std_dev', 0.5)
        
        # Debug
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('log_stats_interval', 5.0)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.ground_z_threshold = self.get_parameter('ground_z_threshold').value
        self.robot_filter_enabled = self.get_parameter('robot_filter_enabled').value
        self.robot_filter_padding = self.get_parameter('robot_filter_padding').value
        self.workspace_bounds = {
            'min_x': self.get_parameter('workspace_min_x').value,
            'max_x': self.get_parameter('workspace_max_x').value,
            'min_y': self.get_parameter('workspace_min_y').value,
            'max_y': self.get_parameter('workspace_max_y').value,
            'min_z': self.get_parameter('workspace_min_z').value,
            'max_z': self.get_parameter('workspace_max_z').value,
        }
        self.publish_debug_markers = self.get_parameter('publish_debug_markers').value
        self.log_stats_interval = self.get_parameter('log_stats_interval').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/collision_filter/markers', 10)
        
        # Subscriber
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            10
        )

        # Statistics
        self.frame_count = 0
        self.total_points_in = 0
        self.total_points_out = 0
        self.create_timer(self.log_stats_interval, self.log_stats)

        # Robot link positions for self-filtering (updated each frame)
        self.robot_links = [
            'base_link',
            'shoulder_link',
            'upper_arm_link',
            'forearm_link',
            'wrist_1_link',
            'wrist_2_link',
            'wrist_3_link',
        ]
        
        self.get_logger().info(
            f'PointCloudCollisionFilter initialized:\n'
            f'  Input: {self.input_topic}\n'
            f'  Output: {self.output_topic}\n'
            f'  Robot frame: {self.robot_frame}\n'
            f'  Ground Z threshold: {self.ground_z_threshold}m\n'
            f'  Robot filter: {self.robot_filter_enabled}\n'
            f'  Workspace: X[{self.workspace_bounds["min_x"]:.1f}, {self.workspace_bounds["max_x"]:.1f}], '
            f'Y[{self.workspace_bounds["min_y"]:.1f}, {self.workspace_bounds["max_y"]:.1f}], '
            f'Z[{self.workspace_bounds["min_z"]:.1f}, {self.workspace_bounds["max_z"]:.1f}]'
        )

    def log_stats(self):
        """Log filtering statistics."""
        if self.frame_count > 0:
            avg_in = self.total_points_in / self.frame_count
            avg_out = self.total_points_out / self.frame_count
            ratio = (avg_out / avg_in * 100) if avg_in > 0 else 0
            self.get_logger().info(
                f'Stats ({self.frame_count} frames): '
                f'in={avg_in:.0f}, out={avg_out:.0f} ({ratio:.1f}%), '
                f'filtered={avg_in-avg_out:.0f}'
            )
            # Reset counters
            self.frame_count = 0
            self.total_points_in = 0
            self.total_points_out = 0

    def get_robot_link_positions(self) -> List[Tuple[str, np.ndarray, float]]:
        """
        Get current robot link positions for self-filtering.
        Returns list of (link_name, position, radius) tuples.
        """
        links = []
        
        for link_name in self.robot_links:
            try:
                # Get transform from robot frame to link
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    link_name,
                    rclpy.time.Time()
                )
                
                pos = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                
                # Approximate link sizes (conservative)
                link_radii = {
                    'base_link': 0.08,
                    'shoulder_link': 0.07,
                    'upper_arm_link': 0.06,
                    'forearm_link': 0.05,
                    'wrist_1_link': 0.04,
                    'wrist_2_link': 0.04,
                    'wrist_3_link': 0.03,
                }
                radius = link_radii.get(link_name, 0.05) + self.robot_filter_padding
                
                links.append((link_name, pos, radius))
                
            except Exception as e:
                self.get_logger().debug(f'Could not get TF for {link_name}: {e}')
                continue
        
        return links

    def filter_ground_plane(self, points: np.ndarray) -> np.ndarray:
        """Remove points below ground threshold (in robot frame)."""
        return points[points[:, 2] >= self.ground_z_threshold]

    def filter_workspace(self, points: np.ndarray) -> np.ndarray:
        """Keep only points within workspace bounds."""
        mask = (
            (points[:, 0] >= self.workspace_bounds['min_x']) &
            (points[:, 0] <= self.workspace_bounds['max_x']) &
            (points[:, 1] >= self.workspace_bounds['min_y']) &
            (points[:, 1] <= self.workspace_bounds['max_y']) &
            (points[:, 2] >= self.workspace_bounds['min_z']) &
            (points[:, 2] <= self.workspace_bounds['max_z'])
        )
        return points[mask]

    def filter_robot_self(self, points: np.ndarray, robot_links: List[Tuple[str, np.ndarray, float]]) -> np.ndarray:
        """Remove points that are inside robot link collision spheres."""
        if not robot_links or not self.robot_filter_enabled:
            return points
        
        # For each point, check distance to all robot links
        keep_mask = np.ones(len(points), dtype=bool)
        
        for link_name, link_pos, radius in robot_links:
            # Calculate distance from each point to link center
            distances = np.linalg.norm(points[:, :3] - link_pos, axis=1)
            # Mark points inside sphere as filtered
            keep_mask &= (distances > radius)
        
        return points[keep_mask]

    def publish_debug_markers(self, robot_links: List[Tuple[str, np.ndarray, float]]):
        """Publish visualization markers for robot collision spheres."""
        if not self.publish_debug_markers:
            return
        
        marker_array = MarkerArray()
        
        for i, (link_name, pos, radius) in enumerate(robot_links):
            marker = Marker()
            marker.header.frame_id = self.robot_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robot_collision_filter'
            marker.id = i
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
            marker.color.a = 0.3  # Transparent
            marker.lifetime.sec = 1  # 1 second lifetime
            marker_array.markers.append(marker)
        
        self.pub_markers.publish(marker_array)

    def cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud."""
        try:
            # Get TF from camera to robot frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().debug(f'TF lookup failed: {e}')
                return

            # Convert point cloud to numpy array
            points_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
            
            if len(points_list) == 0:
                self.get_logger().debug('Empty point cloud received')
                return
            
            points = np.array(points_list)
            original_count = len(points)
            self.total_points_in += original_count

            # Transform points to robot frame
            # Get transform matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z
            
            # Apply transformation to points
            points_homogeneous = np.hstack([points[:, :3], np.ones((len(points), 1))])
            points_transformed = (T @ points_homogeneous.T).T[:, :3]
            
            # Combine with RGB
            if points.shape[1] > 3:
                points_transformed = np.hstack([points_transformed, points[:, 3:]])
            
            # Apply filters
            # 1. Remove ground plane
            points_filtered = self.filter_ground_plane(points_transformed)
            
            # 2. Workspace crop
            points_filtered = self.filter_workspace(points_filtered)
            
            # 3. Robot self-filter (get current robot positions)
            if self.robot_filter_enabled:
                robot_links = self.get_robot_link_positions()
                points_filtered = self.filter_robot_self(points_filtered, robot_links)
                self.publish_debug_markers(robot_links)
            
            filtered_count = len(points_filtered)
            self.total_points_out += filtered_count
            self.frame_count += 1

            # Convert back to PointCloud2
            if filtered_count > 0:
                # Create output message
                header = msg.header
                header.frame_id = self.robot_frame
                
                # Build output point cloud
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                
                # Add RGB field if present
                if points_filtered.shape[1] > 3:
                    fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
                    point_step = 16
                else:
                    point_step = 12
                
                # Create output message
                out_msg = PointCloud2()
                out_msg.header = header
                out_msg.height = 1
                out_msg.width = filtered_count
                out_msg.fields = fields
                out_msg.is_bigendian = False
                out_msg.point_step = point_step
                out_msg.row_step = point_step * filtered_count
                out_msg.is_dense = False
                
                # Convert to bytes
                if points_filtered.shape[1] > 3:
                    data = points_filtered.astype(np.float32).tobytes()
                else:
                    data = points_filtered.astype(np.float32).tobytes()
                
                out_msg.data = data
                
                # Publish
                self.pub_filtered.publish(out_msg)
                
                # Debug logging
                if self.frame_count <= 3:
                    self.get_logger().info(
                        f'Filtered cloud: {original_count} -> {filtered_count} '
                        f'({(filtered_count/original_count*100):.1f}%)'
                    )

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCollisionFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
