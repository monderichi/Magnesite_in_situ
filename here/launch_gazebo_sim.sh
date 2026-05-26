#!/bin/bash
# Standalone launcher for UR robot in Gazebo Sim with MoveIt
# Fixes: Snap environment contamination, build issues, controller timing
# Usage: ./launch_gazebo_sim.sh

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "UR Robot Gazebo Sim + MoveIt Launcher"
echo "========================================="

# Clean up Snap environment variables that interfere with ROS2/RViz
# VS Code Snap sets GTK/GIO paths that cause symbol lookup errors in rviz2
echo "[INFO] Cleaning Snap environment variables..."
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR

# Remove any snap library paths from LD_LIBRARY_PATH
if [[ -n "$LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
fi

# Source ROS environment
echo "[INFO] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "[WARN] Workspace not built. Building spraying_pathways package..."
    cd "$SCRIPT_DIR"
    colcon build --packages-select spraying_pathways
fi

# Source workspace
echo "[INFO] Sourcing workspace..."
cd "$SCRIPT_DIR"
source install/setup.bash

# Clear MoveIt warehouse database to avoid column errors
echo "[INFO] Clearing MoveIt warehouse database..."
rm -f ~/.ros/warehouse_ros.sqlite

# Launch the simulation
echo "[INFO] Launching UR robot with Gazebo Sim and MoveIt..."
echo "========================================="
echo ""

ros2 launch spraying_pathways bringup_v4_gz_sim.launch.py

echo ""
echo "========================================="
echo "Simulation terminated"
echo "========================================="
