# Complete Analysis: bringup_v4.launch.py

## Overview

This document analyzes how `bringup_v4.launch.py` launches a **UR10e robotic arm** with **MoveIt2** motion planning and **Gazebo** simulation, and provides a guide for creating a similar setup for a new robot.

---

## What bringup_v4.launch.py Launches

### 1. Launch Arguments (16 total)

| Argument | Default Value | Purpose |
|----------|---------------|---------|
| `ur_type` | `ur10e` | Robot model (ur3e, ur5e, ur10e, etc.) |
| `safety_limits` | `true` | Enable safety limits controller |
| `safety_pos_margin` | `0.15` | Safety margin for joint limits |
| `safety_k_position` | `20` | Safety controller k-position factor |
| `runtime_config_package` | `ur_simulation_gazebo` | Package with controller configs |
| `controllers_file` | `ur_controllers.yaml` | Controller configuration file |
| `initial_positions_file` | `initial_positions.yaml` | Initial joint positions |
| `description_package` | `spraying_pathways` | Package with URDF/XACRO |
| `description_file` | `my_robot.urdf.xacro` | Main robot description file |
| `prefix` | `""` | Joint name prefix |
| `start_joint_controller` | `true` | Start joint trajectory controller |
| `initial_joint_controller` | `joint_trajectory_controller` | Controller to start |
| `gazebo_gui` | `true` | Show Gazebo GUI |
| `moveit_config_package` | `ur_moveit_config` | MoveIt configuration package |
| `moveit_config_file` | `ur.srdf.xacro` | SRDF semantic description |
| `world_file` | `table_world.world` | Gazebo world file |

### 2. Nodes Launched

#### A. Robot State Publisher
```python
Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"use_sim_time": True}, robot_description],
)
```
- **Purpose**: Publishes TF tree from URDF and joint states
- **Subscribes**: `/joint_states`
- **Publishes**: `/tf`, `/tf_static`, `/robot_description`

#### B. Controller Manager Spawners
```python
# Joint State Broadcaster
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
)

# Joint Trajectory Controller (conditional)
Node(
    package="controller_manager",
    executable="spawner",
    arguments=[initial_joint_controller, "-c", "/controller_manager"],
    condition=IfCondition(start_joint_controller),
)
```
- **Purpose**: Load ROS2 controllers for joint control
- **Controllers**:
  - `joint_state_broadcaster`: Publishes `/joint_states`
  - `joint_trajectory_controller`: Executes motion plans

#### C. Gazebo Entity Spawner
```python
Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=["-entity", "ur", "-topic", "robot_description", 
               "-x", "0.25", "-y", "0", "-z", "0.715"],
)
```
- **Purpose**: Spawns robot in Gazebo at specified position
- **Note**: Uses `/robot_description` topic to get URDF

#### D. Timer Action (Delayed Script)
```python
TimerAction(
    period=10.0,
    actions=[
        ExecuteProcess(
            cmd=["ros2", "run", "spraying_pathways", "transform_camera_pointcloud.py"],
            shell=True,
        )
    ]
)
```
- **Purpose**: Starts point cloud processing after 10s delay

### 3. Included Launch Files

#### A. Gazebo Server
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
    ]),
    launch_arguments={"world": world_file}.items()
)
```
- **Purpose**: Physics simulation server

#### B. Gazebo Client (GUI)
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
    ]),
    condition=IfCondition(gazebo_gui),
)
```
- **Purpose**: Gazebo visualization GUI

#### C. UR MoveIt Launch
```python
IncludeLaunchDescription(
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
```
- **Purpose**: Launches MoveIt move_group and RViz
- **Nodes started**:
  - `move_group` - Core planning node
  - `rviz2` - Visualization
  - `servo_node_main` (optional) - Real-time control

---

## Robot Description Flow

### URDF/XACRO Processing

```
my_robot.urdf.xacro
    ↓ (includes)
ur_description/urdf/ur.urdf.xacro
    ↓ (includes)
ur_description/urdf/ur_macro.xacro (actual robot definition)
    + spraying_pathways/urdf/depth_camera.urdf.xacro (custom sensor)
    + spraying_pathways/urdf/depth_camera_ee.urdf.xacro (end-effector camera)
```

### Key XACRO Arguments Passed

```python
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
    PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
    "safety_limits:=", safety_limits, " ",
    "safety_pos_margin:=", safety_pos_margin, " ",
    "safety_k_position:=", safety_k_position, " ",
    "name:=ur ur_type:=", ur_type, " ",
    "prefix:=", prefix, " ",
    "sim_gazebo:=true ",                    # ← Enables Gazebo mode
    "simulation_controllers:=", initial_joint_controllers, " ",
    "initial_positions_file:=", initial_positions_file_abs, " ",
    # UR-specific calibration files
    "joint_limit_params:=", PathJoinSubstitution([...]), " ",
    "kinematics_params:=", PathJoinSubstitution([...]), " ",
    "physical_params:=", PathJoinSubstitution([...]), " ",
    "visual_params:=", PathJoinSubstitution([...])
])
```

---

## MoveIt Configuration

### Files Used from `ur_moveit_config`

| File | Purpose |
|------|---------|
| `srdf/ur.srdf.xacro` | Semantic robot description (planning groups) |
| `config/kinematics.yaml` | IK solver configuration (KDL) |
| `config/joint_limits.yaml` | Joint limits for planning |
| `config/ompl_planning.yaml` | OMPL planner settings |
| `config/controllers.yaml` | MoveIt controller manager config |
| `rviz/view_robot.rviz` | RViz configuration |

### Planning Group

```xml
<group name="ur_manipulator">
  <chain base_link="base_link" tip_link="tool0" />
</group>
```

### Key MoveIt Nodes

1. **move_group**: Main planning server
   - Action: `/move_group`
   - Plans trajectories using OMPL
   - Interfaces with controllers

2. **rviz2**: Visualization
   - Displays robot model
   - Motion planning plugin
   - Interactive markers

---

## Controller Configuration

### Controller Manager Configuration (`ur_controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    scaled_joint_trajectory_controller:
      type: ur_controllers/ScaledJointTrajectoryController
```

### Joint Trajectory Controller Settings

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
      # ... per-joint constraints
```

---

## Topic Flow

```
┌─────────────────┐
│     Gazebo      │
│   (Physics +    │────┐
│    Sensors)     │    │
└─────────────────┘    │    ┌──────────────────┐
         │             │    │  joint_state_    │
         │ joint_states│    │  broadcaster     │
         ▼             │    └────────┬─────────┘
┌─────────────────┐    │             │
│ /joint_states   │◀───┘             │ /tf, /tf_static
└────────┬────────┘                  ▼
         │                  ┌──────────────────┐
         │                  │ robot_state_     │
         │                  │ publisher        │
         ▼                  └──────────────────┘
┌─────────────────┐
│   move_group    │
│    (MoveIt)     │
└────────┬────────┘
         │ follow_joint_trajectory (action)
         ▼
┌─────────────────┐
│   controller_   │
│   manager       │
└────────┬────────┘
         │ command
         ▼
┌─────────────────┐
│     Gazebo      │
│  (Joint Control)│
└─────────────────┘
```

---

## How to Create a New Robot Setup

### Step 1: Create Robot Description Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

Create directory structure:
```
my_robot_description/
├── urdf/
│   ├── my_robot.urdf.xacro      # Main robot description
│   └── my_robot_macro.xacro     # Robot macro (links, joints)
├── meshes/                       # STL/DAE files
├── config/
│   ├── default_kinematics.yaml   # DH parameters
│   ├── joint_limits.yaml         # Joint limits
│   └── initial_positions.yaml    # Default pose
└── package.xml
```

### Step 2: Create URDF/XACRO

**my_robot_macro.xacro** (define links and joints):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="my_robot" params="
    name
    prefix
    parent
    *origin
    sim_gazebo:=false">

    <!-- Base Link -->
    <link name="${prefix}base_link">
      <visual>
        <geometry>
          <mesh filename="package://my_robot_description/meshes/base_link.dae"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://my_robot_description/meshes/base_link.dae"/>
        </geometry>
      </collision>
    </link>

    <!-- Joint 1 -->
    <joint name="${prefix}joint_1" type="revolute">
      <parent link="${parent}"/>
      <child link="${prefix}link_1"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-3.14" upper="3.14" velocity="1.0" effort="100"/>
    </joint>

    <!-- Link 1 -->
    <link name="${prefix}link_1">
      <visual>
        <geometry>
          <mesh filename="package://my_robot_description/meshes/link_1.dae"/>
        </geometry>
      </visual>
    </link>

    <!-- ... more joints and links ... -->

    <!-- Tool Link -->
    <link name="${prefix}tool0"/>
    <joint name="${prefix}flange-tool0" type="fixed">
      <parent link="${prefix}flange"/>
      <child link="${prefix}tool0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </joint>

    <!-- ros2_control for Gazebo -->
    <xacro:if value="${sim_gazebo}">
      <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
          <parameters>$(arg simulation_controllers)</parameters>
        </plugin>
      </gazebo>
    </xacro:if>

  </xacro:macro>
</robot>
```

**my_robot.urdf.xacro** (main file):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="$(arg name)">
  <xacro:arg name="name" default="my_robot"/>
  <xacro:arg name="prefix" default=""/>
  <xacro:arg name="sim_gazebo" default="false"/>
  <xacro:arg name="simulation_controllers" default=""/>

  <xacro:include filename="$(find my_robot_description)/urdf/my_robot_macro.xacro"/>

  <link name="world"/>
  <xacro:my_robot
    name="$(arg name)"
    prefix="$(arg prefix)"
    parent="world"
    sim_gazebo="$(arg sim_gazebo)">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:my_robot>
</robot>
```

### Step 3: Create MoveIt Configuration

```bash
ros2 pkg create --build-type ament_cmake my_robot_moveit_config
```

Directory structure:
```
my_robot_moveit_config/
├── config/
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── ompl_planning.yaml
│   └── controllers.yaml
├── srdf/
│   └── my_robot.srdf.xacro
├── rviz/
│   └── view_robot.rviz
└── launch/
    └── my_robot_moveit.launch.py
```

**kinematics.yaml**:
```yaml
robot_description_kinematics:
  my_robot_manipulator:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
    kinematics_solver_attempts: 3
```

**my_robot.srdf.xacro**:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:arg name="prefix" default=""/>
  
  <!-- Planning Group -->
  <group name="${prefix}my_robot_manipulator">
    <chain base_link="${prefix}base_link" tip_link="${prefix}tool0"/>
  </group>
  
  <!-- Named States -->
  <group_state name="home" group="${prefix}my_robot_manipulator">
    <joint name="${prefix}joint_1" value="0"/>
    <joint name="${prefix}joint_2" value="0"/>
    <!-- ... -->
  </group_state>
  
  <!-- Disable collisions between adjacent links -->
  <disable_collisions link1="${prefix}base_link" link2="${prefix}link_1" reason="Adjacent"/>
  <!-- ... -->
</robot>
```

**controllers.yaml**:
```yaml
controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - joint_1
    - joint_2
    - joint_3
    - joint_4
    - joint_5
    - joint_6
```

### Step 4: Create Controller Configuration

**my_robot_controllers.yaml**:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
```

### Step 5: Create Launch File

Adapt from `bringup_v4.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
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
    # Launch arguments
    description_package = "my_robot_description"
    description_file = "my_robot.urdf.xacro"
    controllers_file = "my_robot_controllers.yaml"
    moveit_config_package = "my_robot_moveit_config"
    moveit_config_file = "my_robot.srdf.xacro"
    
    # Build robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " sim_gazebo:=true",
        " simulation_controllers:=", PathJoinSubstitution([
            FindPackageShare(description_package), "config", controllers_file
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Joint Trajectory Controller
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    
    # Spawn in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "my_robot", "-topic", "robot_description"],
    )
    
    # Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ]),
        condition=IfCondition(LaunchConfiguration("gazebo_gui")),
    )
    
    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "launch", "my_robot_moveit.launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )
    
    return [
        gzserver,
        gzclient,
        robot_state_publisher_node,
        joint_state_broadcaster,
        joint_trajectory_controller,
        spawn_robot,
        moveit_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
```

### Step 6: Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description my_robot_moveit_config
source install/setup.bash
ros2 launch my_robot_moveit_config my_robot_bringup.launch.py
```

---

## Key Differences for Custom Robots

| Aspect | UR Robot | Custom Robot |
|--------|----------|--------------|
| **Calibration** | Uses YAML kinematics files | Define in URDF or calibration YAML |
| **Safety Limits** | Built into URDF via xacro | Define in joint_limits.yaml |
| **ros2_control** | UR-specific hardware interface | Use generic gazebo_ros2_control |
| **Controllers** | UR controllers package | Standard ros2_controllers |
| **MoveIt Config** | ur_moveit_config package | Custom moveit_config package |

---

## Important Notes

1. **Joint Names**: Must match between URDF, controllers.yaml, and MoveIt config
2. **Planning Group Name**: Used by C++ nodes (e.g., `move_group_interface`)
3. **Command Interfaces**: Use `position` for Gazebo, may differ for real hardware
4. **TF Tree**: Ensure all links are connected properly for kinematics
5. **Gazebo Plugin**: `libgazebo_ros2_control.so` bridges Gazebo and ROS2 controllers

---

## Debugging Tips

1. **Check robot description**: `ros2 topic echo /robot_description | head -20`
2. **Verify joint states**: `ros2 topic echo /joint_states`
3. **Check controllers**: `ros2 control list_controllers`
4. **View TF tree**: `ros2 run tf2_tools view_frames`
5. **Test planning**: Use RViz MotionPlanning plugin
