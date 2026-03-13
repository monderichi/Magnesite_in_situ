#!/bin/bash
set -e

# Clean up Snap environment variables that interfere with ROS2/RViz
# VS Code Snap sets GTK/GIO paths that cause symbol lookup errors in rviz2
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR
# Remove any snap library paths from LD_LIBRARY_PATH
if [[ -n "$LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
fi

# Source ROS environment
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Warning: install/setup.bash not found. Building workspace might be required."
    colcon build --packages-select spraying_pathways
    source install/setup.bash
fi

# Clear MoveIt warehouse database to avoid column errors
rm -f ~/.ros/warehouse_ros.sqlite

# Launch the simulation
echo "Launching UR robot with Gazebo Sim (Ogre rendering) and MoveIt..."
ros2 launch spraying_pathways bringup_v4_gz_sim.launch.py
