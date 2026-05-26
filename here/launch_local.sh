#!/bin/bash

# Script to launch the robot simulation locally without Docker
# This fixes library path conflicts with snap packages

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace
source /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws/install/setup.bash

# Set Gazebo plugin path
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib

# Remove snap libraries from LD_LIBRARY_PATH to avoid conflicts with RViz
if [[ "$LD_LIBRARY_PATH" == *"/snap/"* ]]; then
    echo "Removing snap libraries from LD_LIBRARY_PATH to avoid RViz conflicts..."
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | tr '\n' ':' | sed 's/:$//')
fi

# Change to workspace directory
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws

echo "========================================"
echo "Launching Robot Simulation"
echo "========================================"
echo "This will start:"
echo "  - Gazebo (physics simulation)"
echo "  - MoveIt (motion planning)"
echo "  - RViz (visualization)"
echo "  - UR10e robot"
echo "========================================"
echo ""

# Launch
ros2 launch spraying_pathways bringup_v4.launch.py
