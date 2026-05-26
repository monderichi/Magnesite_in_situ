#!/usr/bin/env python3
"""
RealSense D456 on stand - positioned relative to end effector work position.

Camera is mounted on a separate stand (not on robot).
Position: 20cm X offset, 30cm above end effector working position.
View: Top-down looking at workspace.

Adjust the static transform below based on your actual setup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Camera position relative to robot base_link
    # 
    # Assuming end effector working position is approximately:
    #   x=0.3, y=0.0, z=0.2 (relative to base_link)
    # 
    # Camera position: 20cm X offset, 30cm above end effector
    #   x=0.3 + 0.2 = 0.5
    #   y=0.0
    #   z=0.2 + 0.3 = 0.5
    #
    # ADJUST THESE VALUES based on your actual camera position!
    camera_x = LaunchConfiguration('camera_x', default='0.5')
    camera_y = LaunchConfiguration('camera_y', default='0.0')
    camera_z = LaunchConfiguration('camera_z', default='0.5')
    
    # Top-down view: camera looks down (-Z in camera frame is forward)
    # Roll=PI (upside down), Pitch=0, Yaw=0
    # Or if mounted normally looking down: Roll=0, Pitch=PI/2, Yaw=0
    camera_roll = LaunchConfiguration('camera_roll', default='3.14159')
    camera_pitch = LaunchConfiguration('camera_pitch', default='0.0')
    camera_yaw = LaunchConfiguration('camera_yaw', default='0.0')

    # Static transform from robot base to camera
    # The camera is stationary on a stand, so this doesn't change
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    # Alternative: use robot world frame instead of base_link
    # If your robot is mounted on a table and camera is on the same table,
    # you might want to use 'world' or 'table' frame instead
    
    # RealSense D456 camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[
            # Enable streams
            {'enable_color': True},
            {'enable_depth': True},
            {'enable_infra1': False},
            {'enable_infra2': False},
            
            # Resolution and FPS
            {'rgb_camera.profile': '1280x720x30'},
            {'depth_module.profile': '1280x720x30'},
            
            # Point cloud
            {'pointcloud.enable': True},
            {'pointcloud.ordered_pc': False},
            {'pointcloud.stream_filter': '2'},  # 0=Any, 1=Depth, 2=Color
            {'pointcloud.stream_index_filter': '0'},
            
            # Align depth to color
            {'align_depth.enable': True},
            
            # Filters
            {'spatial_filter.enable': True},
            {'temporal_filter.enable': True},
            {'hole_filling_filter.enable': True},
            
            # Depth settings
            {'depth_module.depth_profile': '1280x720x30'},
            {'depth_module.depth_format': 'Z16'},
            {'clip_distance': 4.0},  # Clip points beyond 4 meters
            
            # TF - use our static transform instead of camera's internal TF
            {'publish_tf': False},  # We publish TF separately above
            {'tf_publish_rate': 0.0},
            
            # Frame IDs
            {'frame_id': 'camera_link'},
            {'base_frame_id': 'camera_link'},
            {'odom_frame_id': 'camera_link'},
            {'imu_optical_frame_id': 'camera_gyro_optical_frame'},
            {'gyro_optical_frame_id': 'camera_gyro_optical_frame'},
            {'accel_optical_frame_id': 'camera_accel_optical_frame'},
            {'color_frame_id': 'camera_color_frame'},
            {'depth_frame_id': 'camera_depth_frame'},
        ],
        output='screen',
        remappings=[
            # Remap to standard topic names
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
            ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ('/camera/depth/color/points', '/camera/depth/color/points'),
        ],
    )

    # Point cloud filter - limits the point cloud to workspace area
    # This helps MoveIt by removing irrelevant points
    pointcloud_filter = Node(
        package='spraying_pathways',
        executable='pointcloud_workspace_filter.py',
        name='pointcloud_filter',
        parameters=[
            {'input_topic': '/camera/depth/color/points'},
            {'output_topic': '/camera/depth/color/points_filtered'},
            # Workspace bounds relative to camera frame
            # X: forward from camera, Y: left/right, Z: up/down
            # For top-down: Z is pointing down, so workspace is negative Z
            {'min_x': -0.5},  # Don't use points behind camera
            {'max_x': 1.0},   # 1 meter in front
            {'min_y': -0.5},  # 0.5m left
            {'max_y': 0.5},   # 0.5m right
            {'min_z': -1.0},  # 1 meter below camera (workspace)
            {'max_z': 0.1},   # slightly above camera (remove table edge)
        ],
        output='screen',
        condition=LaunchConfiguration('filter_pointcloud', default='true').evaluate(),
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.5',
            description='Camera X position relative to base_link'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera Y position relative to base_link'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.5',
            description='Camera Z position relative to base_link'
        ),
        DeclareLaunchArgument(
            'camera_roll',
            default_value='3.14159',
            description='Camera roll (PI = upside down for top-down)'
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.0',
            description='Camera pitch'
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='0.0',
            description='Camera yaw'
        ),
        DeclareLaunchArgument(
            'filter_pointcloud',
            default_value='true',
            description='Enable workspace point cloud filtering'
        ),
        
        # Nodes
        camera_tf,
        realsense_node,
        # pointcloud_filter,  # Uncomment when filter node exists
    ])
