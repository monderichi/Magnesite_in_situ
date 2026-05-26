from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess
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


def launch_setup(context, *args, **kwargs):
    # LaunchConfigurations
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

    # Robot description
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])
    initial_positions_file_abs = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", initial_positions_file
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=ur ur_type:=", ur_type, " ",
        "prefix:=", prefix, " ",
        "sim_gazebo:=true ",
        "simulation_controllers:=", initial_joint_controllers, " ",
        "initial_positions_file:=", initial_positions_file_abs, " ",

        # These 4 lines are essential:
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


    robot_description = {"robot_description": robot_description_content}

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

    spawn_ur = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ur", "-topic", "robot_description", "-x", "0.25", "-y", "0", "-z", "0.715"],
        output="screen",
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
        launch_arguments={"world": world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ]),
        condition=IfCondition(gazebo_gui),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
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
            "use_fake_hardware": "true",
        }.items(),
    )

    transform_camera_pointcloud_script = TimerAction(
        period=10.0,  # Delay in seconds to let the system stabilize
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "spraying_pathways", "transform_camera_pointcloud.py"], 
                shell=True,
                output="screen"
            )
        ]
    )



    return [
        ur_moveit_launch,
        gzserver,
        gzclient,
        robot_state_publisher_node,
        joint_state_broadcaster,
        initial_joint_controller_start,
        initial_joint_controller_stop,
        spawn_ur,
        transform_camera_pointcloud_script,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur_simulation_gazebo"),
        DeclareLaunchArgument("controllers_file", default_value="ur_controllers.yaml"),
        DeclareLaunchArgument("initial_positions_file", default_value=PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", "initial_positions.yaml"
        ])),
        DeclareLaunchArgument("description_package", default_value="spraying_pathways"),
        DeclareLaunchArgument("description_file", default_value="my_robot.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value="\"\""),
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur.srdf.xacro"),
        DeclareLaunchArgument("world_file", default_value="/ros2_ws/src/spraying_pathways/worlds/table_world.world"),
        OpaqueFunction(function=launch_setup),
    ])