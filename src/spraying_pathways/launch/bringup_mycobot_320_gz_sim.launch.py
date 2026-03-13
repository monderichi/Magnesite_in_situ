#!/usr/bin/env python3
"""
Bringup launch for myCobot 320 M5 with Gazebo Sim (Ignition)

This launch file follows the same structure as bringup_v4_gz_sim.launch.py
but is configured for the Elephant Robotics myCobot 320 M5.

Features:
- Gazebo Sim (Ignition) simulation with MoveIt2
- Compatible with spraying_pathways C++ nodes
- Proper startup sequence with event handlers
- Optional go_home_node execution with delay

Usage:
    ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py
    ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py run_go_home:=true
    ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py gazebo_gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    AppendEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
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
    # LaunchConfigurations
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
    run_go_home = LaunchConfiguration("run_go_home")
    go_home_delay = LaunchConfiguration("go_home_delay")

    # Robot description - controllers path
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])
    initial_positions_file_abs = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", initial_positions_file
    ])

    # Build robot description using xacro (Gazebo Sim variant)
    # Note: mycobot_320.urdf.xacro handles both use_gazebo and controllers_file internally
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "robot_name:=", robot_type, " ",
        "prefix:=", prefix, " ",
        "use_gazebo:=true ",
        "controllers_file:=", initial_joint_controllers, " ",
    ])

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    
    initial_joint_controller_start = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller,
                   "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        condition=IfCondition(start_joint_controller),
        output="screen",
    )
    
    initial_joint_controller_stop = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller,
                   "-c", "/controller_manager",
                   "--stopped",
                   "--controller-manager-timeout", "30"],
        condition=UnlessCondition(start_joint_controller),
        output="screen",
    )

    # Gazebo Sim (Ignition) - conditional launch for headless vs GUI mode
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

    # Gazebo Sim spawn using ros_gz_sim create
    spawn_mycobot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mycobot_320",
            "-topic", "robot_description",
            "-x", "0.25",
            "-y", "0",
            "-z", "0.715",
        ],
        output="screen",
    )

    # Bridge for /clock topic (Gazebo Sim uses different topic names)
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # MoveIt configuration - inline (no separate launch file)
    pkg_spraying = get_package_share_directory('spraying_pathways')
    
    # Load SRDF using xacro (like UR does)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "mycobot_320_ur", "mycobot_320_ur.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}
    
    # Load kinematics - match UR format with /**/ros__parameters structure
    kinematics_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'kinematics.yaml')
    import yaml
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    # Extract from ROS2 parameter format: /**/ros__parameters/robot_description_kinematics
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_config.get("/**", {})
                                                      .get("ros__parameters", {})
                                                      .get("robot_description_kinematics", {})
    }
    # DEBUG: Print kinematics config
    kinematics_debug = LogInfo(msg=f"[DEBUG] Kinematics file: {kinematics_file}")
    kinematics_debug2 = LogInfo(msg=f"[DEBUG] Kinematics params: {robot_description_kinematics}")
    
    # Load joint limits - match UR format
    joint_limits_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_config = yaml.safe_load(f)
    # Pass joint_limits directly (UR format has joint_limits as root key)
    robot_description_planning = {"robot_description_planning": joint_limits_config}
    
    # Load OMPL planning - match UR format
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    # Load planner configs from yaml
    ompl_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'ompl_planning.yaml')
    with open(ompl_file, 'r') as f:
        ompl_config = yaml.safe_load(f)
    # Merge the yaml content into move_group config
    ompl_planning_pipeline_config["move_group"].update(ompl_config)
    
    # Load MoveIt controllers
    moveit_controllers_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'moveit_controllers.yaml')
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
            {"publish_robot_description_semantic": True},
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

    # Proper startup sequence:
    # 1. Gazebo starts -> 2. Delayed spawn robot -> 3. Delayed start controllers -> 4. Delayed start MoveIt
    # Using TimerActions instead of OnProcessExit because spawn_mycobot is a persistent Node
    
    # Log startup sequence
    log_starting_gazebo = LogInfo(msg="[Launch] Starting Gazebo Sim...")
    log_spawning_robot = LogInfo(msg="[Launch] Spawning myCobot 320 robot...")
    log_starting_controllers = LogInfo(msg="[Launch] Starting controllers...")
    log_starting_moveit = LogInfo(msg="[Launch] Starting MoveIt...")
    
    # Stage 1: Spawn robot after Gazebo is ready (8s matches working UR10 launch)
    spawn_robot_delayed = TimerAction(
        period=8.0,  # Wait for Gazebo to fully initialize
        actions=[log_spawning_robot, spawn_mycobot]
    )
    
    # Stage 2: Start joint_state_broadcaster FIRST after spawn completes
    # gz_ros2_control auto-loads controllers from YAML, so we need enough delay
    # for it to finish initialization before spawners try to configure them.
    # Spawners MUST be sequential to avoid concurrent configure race condition.
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_mycobot,
            on_exit=[
                TimerAction(
                    period=8.0,  # Give gz_ros2_control + controller_manager time to fully init
                    actions=[
                        log_starting_controllers,
                        joint_state_broadcaster,
                    ]
                )
            ],
        )
    )
    
    # Stage 3: Start arm_controller AFTER joint_state_broadcaster spawner exits
    # Sequential spawning avoids concurrent controller_manager configure conflicts
    delay_arm_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                TimerAction(
                    period=2.0,  # Small delay between controller spawners
                    actions=[
                        initial_joint_controller_start,
                    ]
                )
            ],
        )
    )
    
    # Stage 4: Start MoveIt after arm_controller spawner exits
    delay_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_start,
            on_exit=[
                TimerAction(
                    period=3.0,  # Give controllers time to fully activate
                    actions=[log_starting_moveit, move_group_node],
                )
            ]
        )
    )

    return [
        kinematics_debug,          # DEBUG: Print kinematics file path
        kinematics_debug2,         # DEBUG: Print kinematics params
        log_starting_gazebo,       # Log startup
        gazebo_gui_mode,           # Gazebo Sim with GUI (or headless)
        gazebo_headless_mode,
        gz_bridge,                 # Bridge for /clock
        robot_state_publisher_node,
        spawn_robot_delayed,       # Spawn after Gazebo is ready (8s)
        delay_jsb_after_spawn,     # JSB spawner after spawn exits (8s delay)
        delay_arm_controller_after_jsb,  # arm_controller after JSB exits (2s delay)
        delay_moveit_after_controllers,  # MoveIt after arm_controller exits (3s delay)
        rviz_node,
        go_home_timer,             # go_home with delay
    ]


def generate_launch_description():
    # Set Gazebo resource path for model loading
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    # Build resource paths - models need to be findable
    resource_paths = [
        "/usr/share/gz",
        os.path.join(pkg_spraying, "models"),
        os.path.join(get_package_share_directory("mycobot_description"), ".."),
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
        DeclareLaunchArgument("robot_type", default_value="mycobot_320", description="Robot model type"),
        DeclareLaunchArgument("safety_limits", default_value="true", description="Enable safety limits"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15", description="Safety margin for joint limits"),
        DeclareLaunchArgument("safety_k_position", default_value="20", description="Safety controller k-position factor"),
        DeclareLaunchArgument("runtime_config_package", default_value="spraying_pathways", description="Package with controller configs"),
        DeclareLaunchArgument("controllers_file", default_value="mycobot_320_ur/ros2_controllers.yaml", description="Controller configuration file (UR-compatible)"),
        DeclareLaunchArgument("initial_positions_file", default_value="mycobot_320_ur/initial_positions.yaml", description="Initial joint positions (UR-compatible)"),
        DeclareLaunchArgument("description_package", default_value="mycobot_description", description="Package with URDF/XACRO"),
        DeclareLaunchArgument("description_file", default_value="robots/mycobot_320_ur_compatible.urdf.xacro", description="Main robot description file (UR-compatible version)"),
        DeclareLaunchArgument("prefix", default_value="\"\"", description="Joint name prefix"),
        DeclareLaunchArgument("start_joint_controller", default_value="true", description="Start joint trajectory controller"),
        DeclareLaunchArgument("initial_joint_controller", default_value="arm_controller", description="Initial controller to start"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Show Gazebo GUI"),
        DeclareLaunchArgument("moveit_config_package", default_value="spraying_pathways", description="MoveIt configuration package"),
        DeclareLaunchArgument("world_file", default_value=PathJoinSubstitution([
            FindPackageShare("spraying_pathways"), "worlds", "mycobot_320_world.sdf"
        ]), description="Gazebo Sim world file (SDF format)"),
        DeclareLaunchArgument("run_go_home", default_value="false", description="Run go_home_node after delay"),
        DeclareLaunchArgument("go_home_delay", default_value="15.0", description="Delay before starting go_home_node (seconds)"),
        OpaqueFunction(function=launch_setup),
    ])
