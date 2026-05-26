#!/usr/bin/env python3
"""
Launch file for REAL myCobot 320 M5 robot (NO simulation, NO RViz, NO controller_manager)

This launch file controls the physical myCobot 320 M5 robot using:
- pymycobot driver for hardware communication (provides joint states + trajectory action)
- MoveIt2 for motion planning

This version does NOT use ros2_control since the Python driver handles hardware directly.

Prerequisites:
1. Install pymycobot: pip3 install pymycobot
2. Connect robot via USB (typically /dev/ttyACM0)
3. Fix permissions: sudo chmod 666 /dev/ttyACM0
   Or add user to dialout group: sudo usermod -a -G dialout $USER

Usage:
    ros2 launch spraying_pathways real_robot.launch.py
    ros2 launch spraying_pathways real_robot.launch.py port:=/dev/ttyACM0

Safety:
    - Ensure robot has clear workspace before launching
    - Emergency stop button should be accessible
    - Robot will NOT move until a trajectory is sent
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
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
    # LaunchConfigurations
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    robot_type = LaunchConfiguration("robot_type")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    use_rviz = LaunchConfiguration("use_rviz")

    # Build robot description (NO ros2_control for real robot)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "robot_name:=", robot_type, " ",
        "prefix:=", prefix, " ",
        "use_gazebo:=false ",
    ])

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )

    # Real robot driver node (Python) - handles BOTH joint states AND trajectory execution
    mycobot_driver_node = Node(
        package="spraying_pathways",
        executable="mycobot_driver.py",
        name="mycobot_driver",
        output="screen",
        parameters=[{
            "port": port,
            "baud": baud,
            "robot_type": "mycobot_320",
            "joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            "publish_rate": 50.0,  # Joint state publish rate
            "speed": 80,  # Robot speed 0-100 (fresh mode enables smooth motion)
        }],
    )

    # MoveIt configuration
    pkg_spraying = get_package_share_directory('spraying_pathways')

    # Load SRDF
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "mycobot_320_ur", "mycobot_320_ur.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # Load kinematics
    kinematics_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'kinematics.yaml')
    import yaml
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_config.get("/**", {})
                                                  .get("ros__parameters", {})
                                                  .get("robot_description_kinematics", {})
    }

    # Load joint limits
    joint_limits_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_config = yaml.safe_load(f)
    robot_description_planning = {"robot_description_planning": joint_limits_config}

    # Load OMPL planning
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'ompl_planning.yaml')
    with open(ompl_file, 'r') as f:
        ompl_config = yaml.safe_load(f)
    ompl_planning_pipeline_config["move_group"].update(ompl_config)

    # Load MoveIt controllers - configured to use driver's action server
    moveit_controllers_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'moveit_controllers_real_simple.yaml')
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers_config = yaml.safe_load(f)

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_config.get('moveit_simple_controller_manager', {}),
        "moveit_controller_manager": moveit_controllers_config.get('moveit_controller_manager', ''),
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        # INCREASED TIMEOUTS for real robot execution
        # Allow 10x the planned duration (was 2.0) - real robot moves slower than simulation
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        # Allow 5 seconds extra margin (was 1.0)
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        # Tighter start tolerance for accuracy
        "trajectory_execution.allowed_start_tolerance": 0.05,
        # Keep monitoring enabled
        "trajectory_execution.execution_duration_monitoring": True,
        # Wait longer for trajectory completion
        "trajectory_execution.execution_velocity_scaling": 0.5,  # Slower execution for safety
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
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz node (optional)
    use_rviz_value = use_rviz.perform(context)
    rviz_config_file = os.path.join(pkg_spraying, 'rviz', 'view_robot.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_rviz)
    )

    # Spray Valve controller node (Python)
    spray_valve_node = Node(
        package="spraying_pathways",
        executable="spray_valve_node.py",
        name="spray_valve_controller",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("spray_port"),
            "baud": LaunchConfiguration("spray_baud"),
        }],
    )

    # Startup sequence
    log_starting_driver = LogInfo(msg="[Launch] Starting myCobot 320 driver...")
    log_starting_moveit = LogInfo(msg="[Launch] Starting MoveIt...")
    log_starting_rviz = LogInfo(msg="[Launch] Starting RViz...")
    log_starting_valve = LogInfo(msg="[Launch] Starting Spray Valve controller...")

    delay_moveit = TimerAction(
        period=5.0,
        actions=[log_starting_moveit, move_group_node]
    )
    
    delay_rviz = TimerAction(
        period=7.0,
        actions=[log_starting_rviz, rviz_node]
    )

    return [
        log_starting_driver,
        log_starting_valve,
        robot_state_publisher_node,
        mycobot_driver_node,
        # spray_valve_node,
        delay_moveit,
        delay_rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="Serial port for myCobot 320"),
        DeclareLaunchArgument("baud", default_value="115200", description="Baud rate"),
        DeclareLaunchArgument("robot_type", default_value="mycobot_320", description="Robot model type"),
        DeclareLaunchArgument("description_package", default_value="mycobot_description", description="Package with URDF"),
        DeclareLaunchArgument("description_file", default_value="robots/mycobot_320_ur_compatible.urdf.xacro", description="Robot description file"),
        DeclareLaunchArgument("prefix", default_value="\"\"", description="Joint name prefix"),
        DeclareLaunchArgument("moveit_config_package", default_value="spraying_pathways", description="MoveIt config package"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz for visualization"),
        DeclareLaunchArgument("spray_port", default_value="/dev/ttyACM1", description="Serial port for the spray valve Arduino"),
        DeclareLaunchArgument("spray_baud", default_value="115200", description="Baud rate for the spray valve Arduino"),
        OpaqueFunction(function=launch_setup),
    ])
