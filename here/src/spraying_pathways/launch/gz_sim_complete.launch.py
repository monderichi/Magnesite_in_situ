#!/usr/bin/env python3
"""
Complete launch file for UR robot in Gazebo Sim with MoveIt and RViz

Usage:
    rm -f ~/.ros/warehouse_ros.sqlite
    ros2 launch spraying_pathways gz_sim_complete.launch.py

Note: There will be "jump back in time" warnings from Gazebo Sim - these are normal
and don't affect functionality. The TF buffer clears and recovers automatically.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    ur_type = LaunchConfiguration("ur_type")
    
    controllers_file = PathJoinSubstitution([
        FindPackageShare("spraying_pathways"), "config", "minimal_controllers.yaml"
    ])
    
    # Generate robot description for robot_state_publisher
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=", ur_type,
        " ",
        "sim_ignition:=true",
        " ",
        "simulation_controllers:=", controllers_file,
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot State Publisher - use sim time
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    # Gazebo Sim
    world_file = PathJoinSubstitution([
        FindPackageShare("spraying_pathways"), "worlds", "empty_world_gz_sim.sdf"
    ])
    
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "2", world_file],
        output="screen",
    )
    
    # Bridge for /clock
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    
    # Spawn robot - delayed
    spawn_robot = TimerAction(
        period=3.0,
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
    
    # Controllers
    joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    joint_trajectory_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    # MoveIt launch - use standard ur_moveit.launch.py which has its own RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "prefix": '""',
            "use_sim_time": "true",
            "launch_rviz": "true",
            "launch_servo": "false",
        }.items(),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        
        # Set environment
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", "/usr/share/gz:" + pkg_spraying + "/models"),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", "/usr/share/gz:/usr/share/gazebo:" + pkg_spraying + "/models"),
        
        robot_state_publisher,
        gz_bridge,
        gazebo,
        spawn_robot,
        joint_state_broadcaster,
        joint_trajectory_controller,
        
        # MoveIt + RViz - delayed
        TimerAction(period=12.0, actions=[moveit_launch]),
    ])
