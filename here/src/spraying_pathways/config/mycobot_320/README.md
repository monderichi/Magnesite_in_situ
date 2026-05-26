# myCobot 320 M5 + Gazebo + MoveIt2 Setup

This directory contains configuration files for running the **myCobot 320 M5** robot with Gazebo simulation and MoveIt2 motion planning, compatible with all `spraying_pathways` C++ nodes.

## Overview

The setup provides:
- **Gazebo simulation** with physics
- **MoveIt2** motion planning with OMPL
- **ROS2 Control** for joint trajectory execution
- **Compatibility** with all `spraying_pathways` C++ nodes (uses planning group `ur_manipulator`)

## File Structure

```
spraying_pathways/config/mycobot_320/
├── README.md                          # This file
├── ros2_controllers.yaml              # ROS2 controller configuration
├── moveit_controllers.yaml            # MoveIt controller setup
├── kinematics.yaml                    # KDL kinematics solver config
├── joint_limits.yaml                  # Joint limits for planning
├── ompl_planning.yaml                 # OMPL planner configuration
├── initial_positions.yaml             # Initial joint positions
├── mycobot_320.srdf                   # Semantic robot description (SRDF)
└── view_robot.rviz                    # RViz configuration
```

## Robot Configuration

### Joint Names (Elephant Robotics convention)

| Joint | Name | Limits (rad) |
|-------|------|--------------|
| 1 | `joint2_to_joint1` | -2.93 to 2.93 |
| 2 | `joint3_to_joint2` | -2.35 to 2.35 |
| 3 | `joint4_to_joint3` | -2.53 to 2.53 |
| 4 | `joint5_to_joint4` | -2.53 to 2.53 |
| 5 | `joint6_to_joint5` | -2.93 to 2.93 |
| 6 | `joint6output_to_joint6` | -3.14 to 3.14 |

### Links

```
world
  └── base_link (virtual_joint)
      └── link1 (joint2_to_joint1)
          └── link2 (joint3_to_joint2)
              └── link3 (joint4_to_joint3)
                  └── link4 (joint5_to_joint4)
                      └── link5 (joint6_to_joint5)
                          └── link6 (joint6output_to_joint6)
```

### Planning Groups

| Group Name | Purpose | Chain |
|------------|---------|-------|
| `ur_manipulator` | **Primary planning group** for spraying_pathways C++ nodes | base_link → link6 |
| `arm` | Alternative name for mycobot-specific applications | base_link → link6 |

## Usage

### Basic Launch (Gazebo + MoveIt)

```bash
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py
```

### Options

```bash
# Without Gazebo GUI
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py gazebo_gui:=false

# Without RViz
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py launch_rviz:=false

# Custom world
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py world_file:=/path/to/world.sdf
```

### Full Bringup (with spraying nodes)

```bash
ros2 launch spraying_pathways mycobot_320_bringup.launch.py run_spraying:=true
```

## Running C++ Nodes

Once the simulation is running, you can run any `spraying_pathways` C++ node:

```bash
# Terminal 1: Launch simulation
ros2 launch spraying_pathways mycobot_320_gazebo.launch.py

# Terminal 2: Run a C++ node
ros2 run spraying_pathways go_home_node

# Terminal 3: Run cartesian planner
ros2 run spraying_pathways cartesian_path_planner_cubes_v4_node

# Terminal 4: Run flat fan spraying
ros2 run spraying_pathways flat_fan_spraying_v4_node
```

## Nodes Launched

| Node | Package | Purpose |
|------|---------|---------|
| `gzserver` | gazebo_ros | Physics simulation |
| `gzclient` | gazebo_ros | Gazebo GUI |
| `robot_state_publisher` | robot_state_publisher | TF tree publisher |
| `joint_state_broadcaster` | controller_manager | Publishes /joint_states |
| `arm_controller` | controller_manager | Joint trajectory controller |
| `spawn_entity.py` | gazebo_ros | Spawns robot in Gazebo |
| `move_group` | moveit_ros_move_group | Motion planning server |
| `rviz2` | rviz2 | Visualization |

## Topics

### Published

| Topic | Type | Publisher |
|-------|------|-----------|
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher |
| `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher |
| `/robot_description` | std_msgs/String | robot_state_publisher |
| `/clock` | rosgraph_msgs/Clock | gazebo |

### Actions

| Action | Type | Server |
|--------|------|--------|
| `/move_group` | moveit_msgs/MoveGroupAction | move_group |
| `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | arm_controller |

## Configuration Details

### Kinematics

Uses **KDL** kinematics solver:
- Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- Timeout: 0.05s
- Attempts: 5
- Resolution: 0.005

### Controllers

**Joint Trajectory Controller** (`arm_controller`):
- Type: `joint_trajectory_controller/JointTrajectoryController`
- Command interface: `position`
- State interfaces: `position`, `velocity`
- Update rate: 100 Hz

**Joint State Broadcaster**:
- Type: `joint_state_broadcaster/JointStateBroadcaster`
- Publishes: `/joint_states`

### Planning Pipeline

**OMPL** (Open Motion Planning Library):
- Default planner: `RRTConnectkConfigDefault`
- Available planners: SBL, EST, RRT, RRTConnect, RRT*, PRM, FMT, etc.
- Longest valid segment: 0.005

## Troubleshooting

### Controller not starting

Check controller status:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### MoveIt can't find planning group

Verify the SRDF is loaded correctly:
```bash
ros2 param get /move_group robot_description_semantic
```

### Robot not spawning in Gazebo

Check Gazebo topics:
```bash
ros2 topic echo /robot_description | head -20
```

### C++ node fails with planning group error

Make sure the node uses `"ur_manipulator"`:
```cpp
moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
```

## Comparison with UR10e Setup

| Aspect | UR10e | myCobot 320 |
|--------|-------|-------------|
| Planning group | `ur_manipulator` | `ur_manipulator` (for compatibility) |
| Controller | `joint_trajectory_controller` | `arm_controller` |
| Joint names | `shoulder_pan_joint`, etc. | `joint2_to_joint1`, etc. |
| Base package | `ur_description` | `mycobot_description` |
| MoveIt config | `ur_moveit_config` | Built into `spraying_pathways` |
| Kinematics | YAML calibration files | DH parameters in URDF |

## References

- [Elephant Robotics myCobot 320 Docs](https://www.elephantrobotics.com/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [Gazebo ROS2 Control](https://github.com/ros-controls/gazebo_ros2_control)
