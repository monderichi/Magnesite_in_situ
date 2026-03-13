#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import math

class RotatedPointCloudPublisher(Node):
    
    def __init__(self):
        super().__init__('depth_dip_detector')

        # Subscribe to input point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/transformed_points',
            self.pointcloud_callback,
            10)

        # Publisher for the final problematic points
        self.publisher_problematic_points = self.create_publisher(
            PointCloud2,
            '/problematic_points',
            10)

    
    def generate_waypoints(self): 
        #unordered_points = [(0.8, 0.0), (0.8, 0.4), (1.2, 0.0), (1.2, 0.4)]
        corners = [(0.8, 0.4), (1.2, 0.4), (1.2, 0.0), (0.8, 0.0)]

        spray_width = 0.04
        z_height = 0.4
        robot_base_x = 0.25
        robot_base_y = 0.0

        """
        print(f"corners[0][0]: = {corners[0][0]} , corners[0][1] = {corners[0][1]}")
        print(f"corners[1][0]: = {corners[1][0]} , corners[1][1] = {corners[1][1]}")
        print(f"corners[2][0]: = {corners[2][0]} , corners[2][1] = {corners[2][1]}")
        print(f"corners[3][0]: = {corners[3][0]} , corners[3][1] = {corners[3][1]}")
        """
        
        dx = corners[1][0] - corners[0][0]
        dy = corners[3][1] - corners[0][1]
        #print(f"dx: ={dx}")
        #print(f"dy: ={dy}")

        length = math.hypot(dx, corners[1][1] - corners[0][1])
        height = math.hypot(corners[3][0] - corners[0][0], dy)
        #print(f"length: ={length}")
        #print(f"height: ={height}")

        cols = max(1, int(length // spray_width))
        rows = max(1, int(height // spray_width))
        #print(f"cols: ={cols}")
        #print(f"rows: ={rows}")

        step_x = dx / cols
        step_y = dy / rows

        positions = []

        for i in range(rows):
            for j in range(cols):
                col = j if i % 2 == 0 else cols - j - 1
                x = corners[0][0] + (col + 0.5) * step_x - robot_base_x
                y = corners[0][1] + (i + 0.5) * step_y - robot_base_y
                positions.append((round(x, 4), round(y, 4)))

        return positions

    def pointcloud_callback(self, msg):
        # Read point cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        problematic_points = []
        z_threshold = 0.07
        tolerance = 0.007
        for pt in points:
            if pt[2] < (z_threshold - tolerance):
                #print(f"Low Z point: x={pt[0]:.4f}, y={pt[1]:.4f}, z={pt[2]:.4f}")
                problematic_points.append(pt)

        waypoints = self.generate_waypoints()
        matched_waypoints = set()

        """
        print(f"Total waypoints: {len(waypoints)}")
        print("Waypoints:")
        for wp in waypoints:
            print(f"  x={wp[0]:.4f}, y={wp[1]:.4f}")

        """

        for px, py, _ in problematic_points:
            for wx, wy in waypoints:
                distance = math.sqrt((px - wx) ** 2 + (py - wy) ** 2)
                if distance <= 0.02:
                    matched_waypoints.add((wx, wy))
                    break 

        robot_base_x = 0.25
        robot_base_y = 0.0

        if matched_waypoints:
            
            #print("Matched waypoints for problematic points (within 0.02m):")
            #for wp in sorted(matched_waypoints):
                #print(f"Waypoint: x={wp[0]:.4f}, y={wp[1]:.4f}")
            
            rebase_matched_waypoints = [
                (x + robot_base_x, y + robot_base_y) for (x, y) in matched_waypoints
            ]
            print("Rebased matched waypoints for problematic points (within 0.02m):")
            for wp in sorted(rebase_matched_waypoints):
                print(f"Waypoint: x={wp[0]:.4f}, y={wp[1]:.4f}")
        
        else:
            print("No waypoints matched for problematic points.")

        header_final = msg.header
        header_final.frame_id = 'base_link'
        final_msg = pc2.create_cloud_xyz32(header_final, problematic_points)
        self.publisher_problematic_points.publish(final_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = RotatedPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()