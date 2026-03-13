#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import math

class RotatedPointCloudPublisher(Node):
    
    def __init__(self):
        
        super().__init__('rotated_pointcloud_publisher')
        
        # Subscribe to input point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            10)
        
        # Publisher for final transformed and clipped point cloud
        self.publisher_transformed = self.create_publisher(
            PointCloud2,
            '/transformed_points',
            10)
        
        # Publisher for final transformed and clipped point cloud
        self.publisher_final_points = self.create_publisher(
            PointCloud2,
            '/final_points',
            10)

    def pointcloud_callback(self, msg):  
        # Read point cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # --- Rotation 1: +90° around Y axis ---
        theta_y = math.radians(90)
        R_y = np.array([
            [ math.cos(theta_y), 0, math.sin(theta_y)],
            [ 0,                 1, 0],
            [-math.sin(theta_y), 0, math.cos(theta_y)]
        ])
        
        # --- Rotation 2: -90° around X axis ---
        theta_x = math.radians(-90)
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(theta_x), -math.sin(theta_x)],
            [0, math.sin(theta_x),  math.cos(theta_x)]
        ])
        
        # Combined rotation matrix
        R_initial = R_x @ R_y
        
        # :white_check_mark: Initialize rotated_points list
        rotated_points = []
        
        # Apply initial rotations
        for p in points:
            if len(p) >= 3:
                vec = np.array([p[0], p[1], p[2]])
                rotated = R_initial @ vec
                rotated_points.append(tuple(rotated))
        
        # --- Final transformation: translation + custom matrix ---
        translation = np.array([-1.0, 0.2, 0.75])
        
        # Custom matrix: z→x, y→y, -x→z
        R_custom = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        
        final_points = []
        
        for point in rotated_points:
            translated = np.array(point) + translation
            final = R_custom @ translated
            final_points.append(tuple(final))

        # Publish final clipped cloud
        header_final = msg.header
        header_final.frame_id = 'base_link'
        final_msg = pc2.create_cloud_xyz32(header_final, final_points)
        self.publisher_final_points.publish(final_msg)

        # --- Clipping: keep only points within bounding box ---
        clipped_points = [
            pt for pt in final_points
            if 0.55 <= pt[0] <= 0.95 and 0.0 <= pt[1] <= 0.4
        ]
        
        # Publish final clipped cloud
        header_final = msg.header
        header_final.frame_id = 'base_link'
        final_msg = pc2.create_cloud_xyz32(header_final, clipped_points)
        self.publisher_transformed.publish(final_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = RotatedPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()