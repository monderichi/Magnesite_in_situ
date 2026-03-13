#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM1',
            description='Serial port for the Arduino spray valve'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for Serial communication'
        ),
        Node(
            package='spraying_pathways',
            executable='spray_valve_node.py',
            name='spray_valve_controller',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }]
        )
    ])
