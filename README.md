# arise_upatras_collab Workspace

This repository contains the ROS 2 workspace for controlling the myCobot 320 M5 robotic arm, including simulation models, MoveIt 2 configurations, and motion planning scripts.

## Setup and Installation

Before launching the robot or simulation, ensure that your environment is properly set up and the workspace is built. A convenience script is provided:

```bash
./install_local.sh
```

This will build the required packages using `colcon build`. Note that you must have the ROS 2 Humble environment properly configured on your system. 

If running the real robot, ensure that the `pymycobot` python package is installed:
```bash
pip3 install pymycobot
```

## Subsystems

The project includes standalone launch scripts that automatically handle environment sourcing, background cleanup, and component initialization.

### 1. Launching the Simulation Environment

The simulation runs using Gazebo Sim alongside MoveIt 2 and RViz. The components spawn sequentially to ensure stable controller loading.

To launch the simulated myCobot 320:

```bash
./launch_mycobot320_sim.sh
```

This sequence will:
1. Start Gazebo Sim.
2. Spawn the robot model.
3. Activate the joint state broadcaster and arm trajectory controllers.
4. Launch the MoveIt MoveGroup.
5. Open RViz for motion planning visualization.

### CAD Model

The repository includes a STEP file (`phee_parts.STEP`) containing the 3D model of the robotic arm parts and the experimental setup. This can be used for reference or for creating collision meshes in simulation.

### 2. Launching the Real Robot

To control the physical myCobot 320 hardware, ensure it is powered on, Emergency Stop is accessible, and the USB cable is connected. 

To launch the real robot interface:

```bash
./launch_real_robot.sh
```

By default, the script connects to `/dev/ttyACM0` at `115200` baud. If your device appears on a different port (e.g., `/dev/ttyUSB0`), you can pass the appropriate arguments:

```bash
./launch_real_robot.sh --port /dev/ttyUSB0 --baud 115200
```

If you encounter permission issues connecting to the serial port, the script will attempt to adjust permissions automatically, but you may need to add your user to the `dialout` group as prompted.

### 3. Spray Valve Integration

The project includes integration with an Arduino-controlled spray valve. The Arduino should be connected via USB and listen for '1' to open and '0' to close over Serial (115200 baud).

The `spray_valve_node.py` is automatically launched with the real robot. It exposes a ROS 2 topic and service to control the valve.

**Setup permissions:**
Make sure the serial port has the correct permissions (e.g. `/dev/ttyACM0`):
```bash
sudo chmod 666 /dev/ttyACM0
```

**Usage:**
Once the real robot (or the standalone `spray_valve.launch.py`) is running, you can control the valve from a sourced terminal:

*   **Open the valve:**
    ```bash
    ros2 topic pub --once /spray_valve std_msgs/msg/Bool "{data: true}"
    ```
*   **Close the valve:**
    ```bash
    ros2 topic pub --once /spray_valve std_msgs/msg/Bool "{data: false}"
    ```
*   **Toggle the valve state automatically:**
    ```bash
    ros2 service call /toggle_spray_valve std_srvs/srv/Trigger
    ```

