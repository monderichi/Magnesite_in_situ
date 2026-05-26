#!/usr/bin/env python3
"""
Debug launch file for Gazebo Sim - isolates components for testing

Usage:
    # First terminal - run Gazebo only:
    ign gazebo -r -v 4 /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/src/spraying_pathways/worlds/empty_world_gz_sim.sdf
    
    # Second terminal - spawn robot:
    ros2 launch spraying_pathways gz_sim_debug.launch.py spawn_only:=true
    
    # Or run everything together with delays:
    ros2 launch spraying_pathways gz_sim_debug.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
    AppendEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    spawn_only = LaunchConfiguration("spawn_only")
    
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    # Use minimal controllers for testing
    controllers_file = PathJoinSubstitution([
        FindPackageShare("spraying_pathways"), "config", "minimal_controllers.yaml"
    ])
    
    # Generate robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("spraying_pathways"), "urdf", "my_robot_gz_sim.urdf.xacro"]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=", ur_type,
        " ",
        "sim_ignition:=true",
        " ",
        "sim_gazebo:=false",
        " ",
        "simulation_controllers:=", controllers_file,
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    nodes = []
    
    # Always add robot state publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"use_sim_time": True}, robot_description],
        )
    )
    
    # Gazebo bridge for /clock
    nodes.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )
    )
    
    if spawn_only.perform(context) == "true":
        # Only spawn robot (Gazebo already running)
        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "ur",
                    "-topic", "robot_description",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                ],
                output="screen",
            )
        )
        
        # Controllers with delay
        nodes.append(
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                        output="screen",
                    )
                ]
            )
        )
        nodes.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
                        output="screen",
                    )
                ]
            )
        )
    else:
        # Start Gazebo
        world_file = os.path.join(pkg_spraying, "worlds", "empty_world_gz_sim.sdf")
        nodes.append(
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", "-v", "2", world_file],
                output="screen",
            )
        )
        
        # Spawn with delay
        nodes.append(
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-name", "ur",
                            "-topic", "robot_description",
                            "-x", "0.0",
                            "-y", "0.0",
                            "-z", "0.0",
                        ],
                        output="screen",
                    )
                ]
            )
        )
        
        # Controllers with longer delay
        nodes.append(
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                        output="screen",
                    )
                ]
            )
        )
        nodes.append(
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
                        output="screen",
                    )
                ]
            )
        )
    
    return nodes


def generate_launch_description():
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("spawn_only", default_value="false", description="Only spawn robot (Gazebo already running)"),
        
        # Set environment
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", "/usr/share/gz:" + pkg_spraying + "/models"),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib"),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", "/usr/share/gz:/usr/share/gazebo:" + pkg_spraying + "/models"),
        
        OpaqueFunction(function=launch_setup),
    ])
