from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

def launch_setup(context, *args, **kwargs):
    # Launch configuration variables
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")

    # Include ur_sim_control.launch.py (Gazebo + controllers)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",         # Don't launch duplicate RViz
            "launch_gazebo": "true",        
            "gazebo_gui": "true",          
        }.items(),
    )

    # Include ur_moveit.launch.py (MoveIt + planning)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
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

    package_path = FindPackageShare("spraying_pathways").find("spraying_pathways")
    box_sdf_path = os.path.join(package_path, "models", "box_0_5", "model.sdf")

    spawn_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", box_sdf_path,
            "-entity", "grasp_box",
            "-x", "1.0", "-y", "0.2", "-z", "0.77" # "-x", "0.4", "-y", "0.25", "-z", "0.5" 
        ],
        output="screen"
    )

    wood_table_sdf_path = os.path.join(package_path, "models", "wood_table", "model.sdf")

    spawn_wood_table = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", wood_table_sdf_path,
            "-entity", "wood_table",
            "-x", "0",
            "-y", "0",
            "-z", "0",
            "-R", "0",
            "-P", "0",
            "-Y", "1.5708"  # 90-degree rotation around Z (yaw)
        ],
        output="screen"
    )

    black_table_sdf_path = os.path.join(package_path, "models", "black_table", "model.sdf")

    spawn_black_table = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", black_table_sdf_path,
            "-entity", "black_table",
            "-x", "1.1",
            "-y", "0",
            "-z", "0",
            "-R", "0",
            "-P", "0",
            "-Y", "1.5708"  # 90-degree rotation around Z (yaw)
        ],
        output="screen"
    )

    # Delayed box spawn after black table
    spawn_box_after_black_table = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_black_table,
            on_exit=[spawn_box],
        )
    )

    return [
        ur_control_launch,
        ur_moveit_launch,
        spawn_wood_table,
        spawn_black_table,
        spawn_box_after_black_table  # spawn_box is now delayed until black_table is spawned
    ]


def generate_launch_description():
    declared_arguments = []

    # Robot selection
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type of UR robot (e.g., ur10e, ur5e, etc.).",
            choices=[
                "ur3", "ur3e", "ur5", "ur5e", "ur7e",
                "ur10", "ur10e", "ur12e", "ur16e", "ur20", "ur30"
            ]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable safety limits."
        )
    )

    # Configuration arguments
    declared_arguments.extend([
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_gazebo",
            description="Package with controller config YAML."
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="Controllers YAML file."
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Package with robot URDF/XACRO."
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO file describing robot."
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package."
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="SRDF/XACRO MoveIt semantic file."
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix for multi-robot setups."
        ),
    ])

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])