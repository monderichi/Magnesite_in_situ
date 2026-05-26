#!/usr/bin/env python3
"""
Bringup launch for myCobot 320 M5 - follows bringup_v4.launch.py pattern

This launch file follows the exact same structure as bringup_v4.launch.py
but is configured for the Elephant Robotics myCobot 320 M5.

Features:
- Gazebo simulation with MoveIt2
- Compatible with spraying_pathways C++ nodes
- Optional go_home_node execution with delay

Usage:
    ros2 launch spraying_pathways bringup_mycobot_320_v4.launch.py
    ros2 launch spraying_pathways bringup_mycobot_320_v4.launch.py run_go_home:=true
    ros2 launch spraying_pathways bringup_mycobot_320_v4.launch.py gazebo_gui:=false
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
)
from launch.conditions import IfCondition, UnlessCondition
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
    # LaunchConfigurations - same pattern as bringup_v4
    robot_type = LaunchConfiguration("robot_type")
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
    run_go_home = LaunchConfiguration("run_go_home")
    go_home_delay = LaunchConfiguration("go_home_delay")

    # Robot description - controllers path
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])
    initial_positions_file_abs = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", initial_positions_file
    ])

    # Build robot description using xacro (same pattern as bringup_v4)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=", robot_type, " ",
        "robot_name:=", robot_type, " ",
        "prefix:=", prefix, " ",
        "sim_gazebo:=true ",
        "controllers_file:=", initial_joint_controllers, " ",
        "initial_positions_file:=", initial_positions_file_abs, " ",
        # myCobot 320 specific parameters (similar to UR's calibration files)
        "joint_limit_params:=", PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "joint_limits.yaml"
        ]), " ",
        "kinematics_params:=", PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "kinematics.yaml"
        ]), " ",
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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    initial_joint_controller_start = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    
    initial_joint_controller_stop = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    spawn_mycobot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "mycobot_320", "-topic", "robot_description", 
                   "-x", "0.25", "-y", "0", "-z", "0.715"],
        output="screen",
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
        launch_arguments={"world": world_file, "verbose": "true"}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ]),
        condition=IfCondition(gazebo_gui),
    )

    # MoveIt launch - adapted for myCobot 320
    # Since we don't have a separate moveit launch file, we'll include move_group directly
    # This follows the pattern but uses the config from spraying_pathways
    
    # Load SRDF and configs
    pkg_spraying = get_package_share_directory('spraying_pathways')
    
    srdf_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'mycobot_320.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}
    
    # Load kinematics
    kinematics_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'kinematics.yaml')
    import yaml
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_config.get('robot_description_kinematics', {})}
    
    # Load joint limits
    joint_limits_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_config = yaml.safe_load(f)
    robot_description_planning = {"robot_description_planning": joint_limits_config.get('joint_limits', {})}
    
    # Load OMPL planning
    ompl_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'ompl_planning.yaml')
    with open(ompl_file, 'r') as f:
        ompl_config = yaml.safe_load(f)
    
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_config)
    
    # Load MoveIt controllers
    moveit_controllers_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'moveit_controllers.yaml')
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers_config = yaml.safe_load(f)
    
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_config.get('moveit_simple_controller_manager', {}),
        "moveit_controller_manager": moveit_controllers_config.get('moveit_controller_manager', ''),
    }
    
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_spraying, 'config', 'mycobot_320', 'view_robot.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gazebo_gui),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": True},
        ],
    )

    # go_home_node execution with delay (TimerAction)
    go_home_timer = TimerAction(
        period=go_home_delay,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "spraying_pathways", "go_home_node"],
                shell=True,
                output="screen",
            )
        ],
        condition=IfCondition(run_go_home),
    )

    return [
        move_group_node,      # MoveIt first (like ur_moveit_launch in v4)
        gzserver,             # Then Gazebo
        gzclient,
        robot_state_publisher_node,
        joint_state_broadcaster,
        initial_joint_controller_start,
        initial_joint_controller_stop,
        spawn_mycobot,
        rviz_node,
        go_home_timer,        # go_home with delay
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="mycobot_320", description="Robot model type"),
        DeclareLaunchArgument("safety_limits", default_value="true", description="Enable safety limits"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15", description="Safety margin for joint limits"),
        DeclareLaunchArgument("safety_k_position", default_value="20", description="Safety controller k-position factor"),
        DeclareLaunchArgument("runtime_config_package", default_value="spraying_pathways", description="Package with controller configs"),
        DeclareLaunchArgument("controllers_file", default_value="mycobot_320/ros2_controllers.yaml", description="Controller configuration file"),
        DeclareLaunchArgument("initial_positions_file", default_value="mycobot_320/initial_positions.yaml", description="Initial joint positions"),
        DeclareLaunchArgument("description_package", default_value="mycobot_description", description="Package with URDF/XACRO"),
        DeclareLaunchArgument("description_file", default_value="robots/mycobot_320.urdf.xacro", description="Main robot description file"),
        DeclareLaunchArgument("prefix", default_value="\"\"", description="Joint name prefix"),
        DeclareLaunchArgument("start_joint_controller", default_value="true", description="Start joint trajectory controller"),
        DeclareLaunchArgument("initial_joint_controller", default_value="arm_controller", description="Initial controller to start"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Show Gazebo GUI"),
        DeclareLaunchArgument("moveit_config_package", default_value="spraying_pathways", description="MoveIt configuration package"),
        DeclareLaunchArgument("moveit_config_file", default_value="mycobot_320/mycobot_320.srdf", description="SRDF semantic description"),
        DeclareLaunchArgument("world_file", default_value=PathJoinSubstitution([
            FindPackageShare("spraying_pathways"), "worlds", "table_world.world"
        ]), description="Gazebo world file"),
        DeclareLaunchArgument("run_go_home", default_value="false", description="Run go_home_node after delay"),
        DeclareLaunchArgument("go_home_delay", default_value="15.0", description="Delay before starting go_home_node (seconds)"),
        OpaqueFunction(function=launch_setup),
    ])
