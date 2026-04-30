#!/usr/bin/env python3
"""
Bringup launch for myCobot 320 M5 + RealSense D455 on stand

Camera is on a separate stand (not on robot).
Position: 20cm X offset, 30cm above end effector work position.
View: Top-down looking at workspace.

Modes:
  - use_gazebo:=true   - Simulation with Gazebo
  - use_gazebo:=false  - Real robot with hardware driver

Usage:
  # Simulation mode (default)
  ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py
  
  # Real robot mode
  ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py use_gazebo:=false
  
  # Real robot with custom camera position
  ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py use_gazebo:=false camera_x:=0.6 camera_z:=0.55
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    ExecuteProcess,
    AppendEnvironmentVariable,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    use_gazebo = LaunchConfiguration("use_gazebo")
    run_realsense = LaunchConfiguration("run_realsense")
    
    # Robot hardware (for real robot mode)
    robot_port = LaunchConfiguration("robot_port")
    robot_baud = LaunchConfiguration("robot_baud")
    spray_port = LaunchConfiguration("spray_port")
    spray_baud = LaunchConfiguration("spray_baud")
    
    # Camera position
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    pkg_spraying = get_package_share_directory('spraying_pathways')
    
    # Build robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("mycobot_description"),
            "urdf", "robots", "mycobot_320_ur_compatible.urdf.xacro"
        ]), " ",
        "robot_name:=mycobot_320 ",
        "prefix:=\"\" ",
        "use_gazebo:=", use_gazebo, " ",
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher (always needed)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_gazebo}, robot_description],
    )
    
    # ========== SIMULATION MODE (Gazebo) ==========
    
    gazebo_gui_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 4 --render-engine ogre ",
                PathJoinSubstitution([
                    FindPackageShare("spraying_pathways"), "worlds", "mycobot_320_world.sdf"
                ])
            ],
        }.items(),
        condition=IfCondition(use_gazebo),
    )
    
    spawn_mycobot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mycobot_320",
            "-topic", "robot_description",
            "-x", "0.25", "-y", "0", "-z", "0.715",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    
    # Controllers for simulation
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "30"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager", "--controller-manager-timeout", "30"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    
    # ========== REAL ROBOT MODE ==========
    
    # Real robot driver node (Python) - handles joint states AND trajectory execution
    mycobot_driver_node = Node(
        package="spraying_pathways",
        executable="mycobot_driver.py",
        name="mycobot_driver",
        output="screen",
        parameters=[{
            "port": robot_port,
            "baud": robot_baud,
            "robot_type": "mycobot_320",
            "joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            "publish_rate": 50.0,
            "speed": 80,
        }],
        condition=UnlessCondition(use_gazebo),
    )
    
    # Spray Valve controller node (optional, for real robot)
    spray_valve_node = Node(
        package="spraying_pathways",
        executable="spray_valve_node.py",
        name="spray_valve_controller",
        output="screen",
        parameters=[{
            "port": spray_port,
            "baud": spray_baud,
        }],
        condition=UnlessCondition(use_gazebo),
    )

    # ========== MOVEIT CONFIGURATION ==========
    
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("spraying_pathways"),
            "config", "mycobot_320_ur", "mycobot_320_ur.srdf"
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # Load kinematics
    kinematics_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_config.get("/**", {}).get("ros__parameters", {}).get("robot_description_kinematics", {})
    }
    
    # Load joint limits
    joint_limits_file = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_config = yaml.safe_load(f)
    robot_description_planning = {"robot_description_planning": joint_limits_config}
    
    # OMPL planning
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
    
    # Load MoveIt controllers (different config for sim vs real)
    moveit_controllers_file_sim = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'moveit_controllers.yaml')
    moveit_controllers_file_real = os.path.join(pkg_spraying, 'config', 'mycobot_320_ur', 'moveit_controllers_real_simple.yaml')
    
    # Read both configs and merge (use real config values for real robot)
    with open(moveit_controllers_file_sim, 'r') as f:
        moveit_controllers_config_sim = yaml.safe_load(f)
    with open(moveit_controllers_file_real, 'r') as f:
        moveit_controllers_config_real = yaml.safe_load(f)
    
    # Choose config based on mode
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_config_real.get('moveit_simple_controller_manager', {}) if use_gazebo.perform(context) == 'false' else moveit_controllers_config_sim.get('moveit_simple_controller_manager', {}),
        "moveit_controller_manager": moveit_controllers_config_real.get('moveit_controller_manager', '') if use_gazebo.perform(context) == 'false' else moveit_controllers_config_sim.get('moveit_controller_manager', ''),
    }
    
    # Trajectory execution parameters (different for real robot)
    trajectory_execution_sim = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    
    trajectory_execution_real = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,  # More time for real robot
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.execution_velocity_scaling": 0.5,
    }
    
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    # Sensors configuration for point cloud obstacle avoidance
    sensors_config = {
        "sensors": ["pointcloud_sensor"],
        "pointcloud_sensor": {
            "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
            "point_cloud_topic": "/camera/camera/depth/color/points_self_filtered",
            "max_range": 1.2,           # ignore points beyond 1.2 m (myCobot workspace)
            "point_subsample": 3,       # use every 3rd point (perf vs accuracy)
            "padding_offset": 0.03,     # 3 cm safety buffer around obstacles
            "padding_scale": 1.0,
            "max_update_rate": 5.0,     # refresh Octomap at 5 Hz
            # "filtered_cloud_topic" is intentionally omitted to avoid duplicating the point cloud
            # and to fix the RELIABILITY_QOS_POLICY mismatch with RViz.
        }
    }

    # Octomap configuration - REQUIRED to activate MoveIt's 3D collision map.
    # Without octomap_frame + octomap_resolution, sensors_config is silently ignored.
    octomap_config = {
        "octomap_frame": "base_link",  # fixed frame where the Octomap is stored
        "octomap_resolution": 0.02,    # 2 cm voxels (finer = better obstacle detection)
        "max_range": 1.2,
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
            trajectory_execution_real if use_gazebo.perform(context) == 'false' else trajectory_execution_sim,
            planning_scene_monitor_parameters,
            sensors_config,
            octomap_config,
            {"use_sim_time": use_gazebo},
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz - Use minimal config to avoid GPU/OpenGL crashes
    # PointCloud2 can be added manually after camera starts
    rviz_config_file = os.path.join(pkg_spraying, 'rviz', 'mycobot_320_realsense_minimal.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_gazebo},
        ],
    )

    # ========== REALSENSE D455 CAMERA ==========
    
    # Static transform from robot base to camera (on stand)
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
        condition=IfCondition(run_realsense),
    )

    # RealSense D455 node - Conservative settings for USB stability
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[
            # Device settings - you have a D456, not D455
            {'device_type': 'D456'},
            {'initial_reset': True},  # Reset camera on startup to clear any stuck state
            {'reconnect_timeout': 6.0},
            {'wait_for_device_timeout': 10.0},
            
            # Enable only essential streams
            {'enable_color': True},
            {'enable_depth': True},
            {'enable_infra1': False},
            {'enable_infra2': False},
            {'enable_accel': False},
            {'enable_gyro': False},
            
            # Conservative profiles to reduce USB bandwidth
            # Lower resolution = less USB bandwidth = more stable
            # Using 480x270 at 6fps for maximum stability on USB 3.2
            {'rgb_camera.profile': '480x270x6'},  # Very low bandwidth for stability
            {'depth_module.profile': '480x270x6'},
            
            # Point cloud
            {'pointcloud.enable': True},
            {'pointcloud.ordered_pc': False},
            {'pointcloud.stream_filter': 0},
            {'pointcloud.stream_profile': 2},  # 2: color mapping
            {'pointcloud.stream_index_filter': 0},
            
            # Align depth to color so we can extract 3D coordinates for the tracked rocks
            {'align_depth.enable': True},
            
            # Minimal filters to reduce CPU load
            {'spatial_filter.enable': False},
            {'temporal_filter.enable': False},
            {'hole_filling_filter.enable': False},
            {'decimation_filter.enable': True},  # Reduces point cloud density, saves bandwidth
            
            {'clip_distance': 2.0},
            
            # TF settings
            {'publish_tf': False},
            {'tf_publish_rate': 0.0},
            {'frame_id': 'camera_link'},
            {'base_frame_id': 'camera_link'},
            {'color_frame_id': 'camera_color_frame'},
            {'depth_frame_id': 'camera_depth_frame'},
            {'color_optical_frame_id': 'camera_color_optical_frame'},
            {'depth_optical_frame_id': 'camera_depth_optical_frame'},
        ],
        condition=IfCondition(run_realsense),
        output='screen',
    )

    # ========== STARTUP SEQUENCE ==========
    
    log_mode = LogInfo(msg="[Launch] Mode: SIMULATION (Gazebo)")
    log_mode_real = LogInfo(msg="[Launch] Mode: REAL ROBOT")
    log_starting_gazebo = LogInfo(msg="[Launch] Starting Gazebo Sim...")
    log_spawning_robot = LogInfo(msg="[Launch] Spawning robot in Gazebo...")
    log_starting_controllers = LogInfo(msg="[Launch] Starting controllers...")
    log_starting_driver = LogInfo(msg="[Launch] Starting myCobot driver...")
    log_starting_moveit = LogInfo(msg="[Launch] Starting MoveIt...")
    log_starting_camera = LogInfo(msg="[Launch] Starting RealSense D455 camera...")
    
    # Simulation startup sequence
    spawn_robot_delayed = TimerAction(
        period=8.0,
        actions=[log_spawning_robot, spawn_mycobot],
        condition=IfCondition(use_gazebo),
    )
    
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_mycobot,
            on_exit=[
                TimerAction(
                    period=8.0,
                    actions=[log_starting_controllers, joint_state_broadcaster],
                )
            ],
        ),
        condition=IfCondition(use_gazebo),
    )
    
    delay_arm_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[arm_controller]
                )
            ],
        ),
        condition=IfCondition(use_gazebo),
    )
    
    delay_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[log_starting_moveit, move_group_node],
                )
            ]
        ),
        condition=IfCondition(use_gazebo),
    )
    
    # Real robot startup sequence (faster, no Gazebo/controllers)
    moveit_real_delay = TimerAction(
        period=5.0,
        actions=[log_starting_moveit, move_group_node],
        condition=UnlessCondition(use_gazebo),
    )
    
    # RViz starts after MoveIt
    rviz_delay = TimerAction(
        period=7.0,
        actions=[rviz_node],
        condition=IfCondition(use_gazebo),
    )
    
    rviz_delay_real = TimerAction(
        period=7.0,
        actions=[rviz_node],
        condition=UnlessCondition(use_gazebo),
    )
    
    # Additional static transforms for RealSense internal frames
    # These are needed when publish_tf=False in the RealSense node
    camera_color_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_frame',
        ],
        condition=IfCondition(run_realsense),
    )
    
    camera_depth_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_frame_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_frame',
        ],
        condition=IfCondition(run_realsense),
    )
    
    # Optical frames - standard camera optical frame convention
    # Optical frame: Z forward, X right, Y down (rotated -90° Y then -90° Z from camera frame)
    camera_color_optical_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_frame_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'camera_color_frame',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
        condition=IfCondition(run_realsense),
    )
    
    camera_depth_optical_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_frame_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'camera_depth_frame',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
        condition=IfCondition(run_realsense),
    )

    # Camera starts last (with all TF frames)
    # Longer delay (20s) to allow USB to stabilize after robot initialization
    delay_camera = TimerAction(
        period=20.0,
        actions=[
            log_starting_camera,
            camera_tf,
            camera_color_frame_tf,
            camera_depth_frame_tf,
            camera_color_optical_frame_tf,
            camera_depth_optical_frame_tf,
            realsense_node,
        ],
    )
    
    # Optional: Python self-filter node for additional filtering
    # MoveIt's built-in self-filter should handle this, but the Python version
    # can run alongside for debugging (publishes markers showing collision volumes)
    self_filter_node = Node(
        package='spraying_pathways',
        executable='robot_self_filter.py',
        name='robot_self_filter',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/color/points',
            'output_topic': '/camera/camera/depth/color/points_self_filtered',
            'robot_frame': 'base_link',
            'robot_filter_padding': 0.04,
        }],
        condition=IfCondition(run_realsense),
    )

    return [
        log_mode,
        log_mode_real,
        gazebo_gui_mode,
        gz_bridge,
        robot_state_publisher_node,
        spawn_robot_delayed,
        delay_jsb_after_spawn,
        delay_arm_controller_after_jsb,
        delay_moveit_after_controllers,
        mycobot_driver_node,  # Only starts in real mode (condition applied)
        # spray_valve_node,     # Only starts in real mode (condition applied)
        moveit_real_delay,
        rviz_delay,
        rviz_delay_real,
        delay_camera,
        self_filter_node,
    ]


def generate_launch_description():
    pkg_spraying = get_package_share_directory("spraying_pathways")
    
    # Build resource paths for Gazebo
    resource_paths = [
        "/usr/share/gz",
        os.path.join(pkg_spraying, "models"),
        os.path.join(get_package_share_directory("mycobot_description"), ".."),
    ]
    
    set_env_gz_resource = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        ":".join(resource_paths)
    )
    
    set_env_plugin = AppendEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )

    return LaunchDescription([
        set_env_gz_resource,
        set_env_plugin,
        DeclareLaunchArgument("use_gazebo", default_value="false", description="Use Gazebo simulation (false = real robot)"),
        DeclareLaunchArgument("robot_port", default_value="/dev/ttyACM0", description="Serial port for real robot"),
        DeclareLaunchArgument("robot_baud", default_value="115200", description="Baud rate for real robot"),
        DeclareLaunchArgument("spray_port", default_value="/dev/ttyACM1", description="Serial port for spray valve"),
        DeclareLaunchArgument("spray_baud", default_value="115200", description="Baud rate for spray valve"),
        DeclareLaunchArgument("run_realsense", default_value="true", description="Launch RealSense camera"),
        DeclareLaunchArgument("camera_x", default_value="0.145", description="Camera X position relative to base_link (forward)"),
        DeclareLaunchArgument("camera_y", default_value="0.245", description="Camera Y position relative to base_link (left)"),
        DeclareLaunchArgument("camera_z", default_value="0.600", description="Camera Z position relative to base_link (up)"),
        DeclareLaunchArgument("camera_roll", default_value="0.0", description="Camera roll (0 = normal)"),
        DeclareLaunchArgument("camera_pitch", default_value="1.5708", description="Camera pitch (PI/2 = 90deg = looking down for top-down view)"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0", description="Camera yaw"),
        OpaqueFunction(function=launch_setup),
    ])
