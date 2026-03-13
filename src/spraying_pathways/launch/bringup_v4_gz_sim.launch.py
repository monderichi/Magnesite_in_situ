#!/usr/bin/env python3
"""
Bringup launch for UR robot with Gazebo Sim (Ignition) - follows bringup_v4.launch.py pattern

This launch file follows the exact same structure as bringup_v4.launch.py
but uses Gazebo Sim (newer version) instead of Gazebo Classic.

Differences from bringup_v4.launch.py:
- Uses ros_gz_sim instead of gazebo_ros
- Sets GZ_SIM_RESOURCE_PATH environment variable
- Uses 'ros_gz_sim create' for spawning robot
- Sets 'gazebo_version:=sim' for xacro processing

Usage:
    ros2 launch spraying_pathways bringup_v4_gz_sim.launch.py
    ros2 launch spraying_pathways bringup_v4_gz_sim.launch.py ur_type:=ur10e
    ros2 launch spraying_pathways bringup_v4_gz_sim.launch.py gazebo_gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    AppendEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # LaunchConfigurations - same as bringup_v4
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    # Robot description - controllers path
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])
    initial_positions_file_abs = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", initial_positions_file
    ])

    # Build robot description using xacro (Gazebo Sim variant)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "sim_ignition:=true "
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=ur ur_type:=", ur_type, " ",
        "prefix:=", prefix, " ",
        "sim_gazebo:=false ",
        "simulation_controllers:=", initial_joint_controllers, " ",
        "joint_limit_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"
        ]), " ",
        "kinematics_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"
        ]), " ",
        "physical_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"
        ]), " ",
        "visual_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"
        ])
    ])

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Nodes - same pattern as bringup_v4
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )
    
    initial_joint_controller_start = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        condition=IfCondition(start_joint_controller),
    )
    
    initial_joint_controller_stop = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped",
                   "--controller-manager-timeout", "30"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Gazebo Sim (instead of gzserver/gzclient)
    # Use conditional launch for headless vs GUI mode
    # GUI mode: just run with world file
    gazebo_gui_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 4 --render-engine ogre ", world_file],
        }.items(),
        condition=IfCondition(gazebo_gui),
    )
    
    # Headless mode: run server only
    gazebo_headless_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": ["-r -s ", world_file],
        }.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Gazebo Sim spawn - wrapped in Timer to ensure Gazebo is ready
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur",
            "-topic", "robot_description",
            "-x", "0.25",
            "-y", "0",
            "-z", "0.715",
        ],
        output="screen",
    )

    # Optional: Bridge for additional topics if needed
    # Gazebo Sim uses different topic names than Classic
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # MoveIt launch - use simulation, not fake hardware
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "launch", "ur_moveit.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "launch_servo": "false",  # Disable servo in simulation
        }.items(),
    )

    # Proper startup sequence using event handlers:
    # 1. Gazebo starts → 2. Spawn robot → 3. After spawn completes, start controllers
    
    # Delayed spawn to ensure Gazebo is ready
    spawn_robot_delayed = TimerAction(
        period=8.0,  # Wait for Gazebo to fully initialize
        actions=[spawn_robot]
    )
    
    # Start controllers AFTER spawn_robot process exits successfully
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,  # Small extra delay for Gazebo to register the entity
                    actions=[
                        joint_state_broadcaster,
                        initial_joint_controller_start,
                        initial_joint_controller_stop,
                    ]
                )
            ],
        )
    )
    
    # Start MoveIt after joint_state_broadcaster is ready
    delay_moveit_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[ur_moveit_launch],
        )
    )

    return [
        gazebo_gui_mode,           # Gazebo Sim with GUI (or headless)
        gazebo_headless_mode,
        gz_bridge,                 # Bridge for /clock
        robot_state_publisher_node,
        spawn_robot_delayed,       # Spawn after Gazebo is ready
        delay_controllers_after_spawn,  # Controllers after spawn completes
        delay_moveit_after_jsb,    # MoveIt after controllers are up
    ]


def generate_launch_description():
    # Set Gazebo resource path for model loading
    # This helps Gazebo Sim find models
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    # Build resource paths - models need to be findable
    resource_paths = [
        "/usr/share/gz",
        os.path.join(pkg_spraying, "models"),
        os.path.join(get_package_share_directory("ur_description"), ".."),
    ]
    
    set_env_gz_resource = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        ":".join(resource_paths)
    )
    
    # Also set GZ_SIM_SYSTEM_PLUGIN_PATH if needed
    set_env_plugin = AppendEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )

    return LaunchDescription([
        set_env_gz_resource,  # Set Gazebo Sim resource path
        set_env_plugin,       # Set plugin path
        DeclareLaunchArgument("ur_type", default_value="ur10e", description="Robot model (ur3e, ur5e, ur10e, etc.)"),
        DeclareLaunchArgument("safety_limits", default_value="true", description="Enable safety limits"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15", description="Safety margin for joint limits"),
        DeclareLaunchArgument("safety_k_position", default_value="20", description="Safety controller k-position factor"),
        DeclareLaunchArgument("runtime_config_package", default_value="spraying_pathways", description="Package with controller configs"),
        DeclareLaunchArgument("controllers_file", default_value="minimal_controllers.yaml", description="Controller configuration file"),
        DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml",
            description="Initial joint positions file name"),
        DeclareLaunchArgument("description_package", default_value="spraying_pathways", description="Package with URDF/XACRO"),
        DeclareLaunchArgument("description_file", default_value="my_robot_gz_sim.urdf.xacro", description="URDF file for Gazebo Sim"),
        DeclareLaunchArgument("prefix", default_value="\"\"", description="Joint name prefix"),
        DeclareLaunchArgument("start_joint_controller", default_value="true", description="Start joint trajectory controller"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller", description="Initial controller to start"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Show Gazebo GUI"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config", description="MoveIt configuration package"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur.srdf.xacro", description="SRDF semantic description"),
        DeclareLaunchArgument("world_file", default_value=PathJoinSubstitution([
            FindPackageShare("spraying_pathways"), "worlds", "empty_world_gz_sim.sdf"
        ]), description="Gazebo Sim world file (SDF format)"),
        OpaqueFunction(function=launch_setup),
    ])
