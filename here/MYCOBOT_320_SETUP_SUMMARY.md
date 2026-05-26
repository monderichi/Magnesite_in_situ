# myCobot 320 M5 + Gazebo + MoveIt2 Setup - Complete Summary

## Overview

This setup brings the **myCobot 320 M5** (6-DOF robotic arm from Elephant Robotics) to work with:
- **Gazebo** physics simulation
- **MoveIt2** motion planning  
- **spraying_pathways** C++ nodes (same code used with UR10e)

## Key Design Decisions

### 1. Planning Group Name: `ur_manipulator`

**Why?** All C++ nodes in `spraying_pathways` use planning group `"ur_manipulator"`:

```cpp
// From cartesian_path_planner_cubes_v4.cpp, flat_fan_spraying_v4.cpp, etc.
moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
```

The SRDF defines **both** planning groups for compatibility:
- `ur_manipulator` - Primary group for spraying_pathways nodes
- `arm` - Alternative name for mycobot-specific applications

### 2. Joint Names: Elephant Robotics Convention

| Joint | Name | UR10e Equivalent |
|-------|------|------------------|
| 1 | `joint2_to_joint1` | `shoulder_pan_joint` |
| 2 | `joint3_to_joint2` | `shoulder_lift_joint` |
| 3 | `joint4_to_joint3` | `elbow_joint` |
| 4 | `joint5_to_joint4` | `wrist_1_joint` |
| 5 | `joint6_to_joint5` | `wrist_2_joint` |
| 6 | `joint6output_to_joint6` | `wrist_3_joint` |

### 3. Controller Names

| UR10e | myCobot 320 |
|-------|-------------|
| `joint_trajectory_controller` | `arm_controller` |
| `joint_state_broadcaster` | `joint_state_broadcaster` (same) |

## Created Files

### Configuration Files

```
spraying_pathways/config/mycobot_320/
├── ros2_controllers.yaml              # ROS2 control configuration
├── moveit_controllers.yaml            # MoveIt ↔ ROS2 control bridge
├── kinematics.yaml                    # KDL kinematics solver
├── joint_limits.yaml                  # Joint limits for planning
├── ompl_planning.yaml                 # OMPL planner configs
├── initial_positions.yaml             # Default spawn pose
├── mycobot_320.srdf                   # Semantic robot description
├── view_robot.rviz                    # RViz configuration
└── README.md                          # Detailed documentation
```

### Launch Files

```
spraying_pathways/launch/
├── mycobot_320_gazebo.launch.py       # Basic launch (Gazebo + MoveIt)
└── mycobot_320_bringup.launch.py      # Full bringup with spraying nodes
```

### Utility Scripts

```
spraying_pathways/scripts/
└── test_mycobot_setup.sh              # Validation script
```

## Quick Start

### 1. Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Build only the necessary packages
colcon build --packages-select spraying_pathways

# Or build everything
colcon build

source install/setup.bash
```

### 2. Validate Setup

```bash
# Run validation script
ros2 run spraying_pathways test_mycobot_setup.sh
```

### 3. Launch the Robot

**Basic launch (Gazebo + MoveIt):**
```bash
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py
```

**Options:**
```bash
# Without Gazebo GUI
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py gazebo_gui:=false

# Without RViz
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py launch_rviz:=false

# Custom world
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py world_file:=/path/to/world.sdf
```

### 4. Run C++ Nodes

Once simulation is running, in new terminals:

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Go home position
ros2 run spraying_pathways go_home_node

# Cartesian path planner
ros2 run spraying_pathways cartesian_path_planner_cubes_v4_node

# Flat fan spraying
ros2 run spraying_pathways flat_fan_spraying_v4_node

# Safety monitor
ros2 run spraying_pathways safety_stop_on_unknown_node
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAUNCH SEQUENCE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│  1. Load URDF/XACRO (mycobot_320.urdf.xacro)                               │
│  2. Start Gazebo (gzserver + gzclient)                                     │
│  3. Start robot_state_publisher                                            │
│  4. Spawn robot in Gazebo                                                  │
│  5. Start controllers (joint_state_broadcaster, arm_controller)            │
│  6. Start MoveIt (move_group)                                              │
│  7. Start RViz                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │ /joint_states
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROS2 CONTROL STACK                                │
│  ┌─────────────────────────┐    ┌─────────────────────────────────────────┐ │
│  │ joint_state_broadcaster │    │ arm_controller                          │ │
│  │   (sensor interface)    │    │  (joint_trajectory_controller)          │ │
│  └───────────┬─────────────┘    └──────────────────┬──────────────────────┘ │
│              │                                     │                         │
│              │ Publishes                           │ Action Server           │
│              ▼                                     ▼                         │
│       /joint_states                     /arm_controller/follow_joint_trajectory
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MOVEIT2 STACK                                  │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │ move_group (moveit_ros_move_group)                                     │ │
│  │  ├── Planning group: "ur_manipulator"                                  │ │
│  │  ├── Kinematics: KDL plugin                                            │ │
│  │  ├── Planner: OMPL (RRTConnect default)                                │ │
│  │  └── Controllers: moveit_simple_controller_manager                     │ │
│  └────────────────────────────┬───────────────────────────────────────────┘ │
│                               │                                              │
│                               │ Action: /move_group                          │
│                               ▼                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │ spraying_pathways C++ Nodes                                            │ │
│  │  ├── go_home_node                                                      │ │
│  │  ├── cartesian_path_planner_cubes_v4_node                              │ │
│  │  ├── flat_fan_spraying_v4_node                                         │ │
│  │  └── ...                                                               │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Configuration Details

### Kinematics Solver (KDL)

```yaml
robot_description_kinematics:
  ur_manipulator:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
    position_only_ik: false
    kinematics_solver_attempts: 5
```

### Joint Limits

```yaml
joint_limits:
  joint2_to_joint1:
    has_velocity_limits: true
    max_velocity: 2.792527
    has_acceleration_limits: true
    max_acceleration: 5.0
    has_position_limits: true
    min_position: -2.93
    max_position: 2.93
  # ... (5 more joints)
```

### ROS2 Controllers

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint2_to_joint1
      - joint3_to_joint2
      - joint4_to_joint3
      - joint5_to_joint4
      - joint6_to_joint5
      - joint6output_to_joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### OMPL Planning

Default planner: **RRTConnectkConfigDefault**

Available planners:
- RRT, RRTConnect, RRT*
- PRM, PRM*
- SBL, EST, KPIECE
- FMT, BFMT
- And more...

## Topics and Services

### Published Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher |
| `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher |
| `/robot_description` | std_msgs/String | robot_state_publisher |
| `/clock` | rosgraph_msgs/Clock | gazebo |
| `/display_planned_path` | moveit_msgs/DisplayTrajectory | move_group |

### Services

| Service | Type | Server |
|---------|------|--------|
| `/move_group/plan_kinematic_path` | moveit_msgs/GetMotionPlan | move_group |
| `/move_group/compute_cartesian_path` | moveit_msgs/GetCartesianPath | move_group |
| `/controller_manager/list_controllers` | controller_manager_msgs/ListControllers | controller_manager |

### Actions

| Action | Type | Server |
|--------|------|--------|
| `/move_group` | moveit_msgs/MoveGroupAction | move_group |
| `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | arm_controller |

## Troubleshooting

### Robot doesn't appear in Gazebo

```bash
# Check robot description
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /robot_description | head -20

# Check Gazebo topics
ros2 topic list | grep gazebo
```

### Controllers not starting

```bash
# List controllers
ros2 control list_controllers

# Check controller_manager
ros2 node info /controller_manager

# Manually load controller
ros2 control load_controller arm_controller
ros2 control set_controller_state arm_controller active
```

### MoveIt can't find planning group

```bash
# Check SRDF is loaded
ros2 param get /move_group robot_description_semantic

# List planning groups
ros2 service call /move_group/get_planning_scene moveit_msgs/GetPlanningScene
```

### C++ node fails

Make sure the node uses the correct planning group:
```cpp
// This MUST be "ur_manipulator" for compatibility
move_group = MoveGroupInterface(node, "ur_manipulator");
```

## Comparison: UR10e vs myCobot 320

| Feature | UR10e | myCobot 320 |
|---------|-------|-------------|
| **Payload** | 10 kg | 1 kg |
| **Reach** | 1300 mm | 350 mm |
| **DOF** | 6 | 6 |
| **Planning Group** | `ur_manipulator` | `ur_manipulator` (same!) |
| **Base Package** | `ur_description` | `mycobot_description` |
| **Controller** | `joint_trajectory_controller` | `arm_controller` |
| **Kinematics** | YAML calibration files | DH in URDF |
| **Sim Speed** | Real-time | Real-time |

## Next Steps

1. **Test basic motion** in RViz using the MotionPlanning plugin
2. **Run go_home_node** to verify trajectory execution
3. **Test cartesian_path_planner** for spraying path planning
4. **Integrate sensors** (depth cameras, etc.)
5. **Add safety monitoring** with safety_stop_on_unknown_node

## References

- **Elephant Robotics:** https://www.elephantrobotics.com/
- **myCobot GitHub:** https://github.com/monderichi/Elephant_320m5_moveit2
- **MoveIt 2 Docs:** https://moveit.picknik.ai/
- **ROS2 Control:** https://control.ros.org/
- **Gazebo:** https://gazebosim.org/

## Support

For issues with:
- **Robot model**: Check `mycobot_description` package
- **MoveIt config**: Check `spraying_pathways/config/mycobot_320/`
- **C++ nodes**: Check `spraying_pathways/src/`
- **Launch files**: Check `spraying_pathways/launch/`

Run validation:
```bash
ros2 run spraying_pathways test_mycobot_setup.sh
```
