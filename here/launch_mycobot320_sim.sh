#!/bin/bash
# Standalone launcher for myCobot 320 M5 in Gazebo Sim with MoveIt
# Fixes: Snap environment contamination, build issues, controller timing
# Usage: ./launch_mycobot320_sim.sh

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "myCobot 320 M5 Gazebo Sim + MoveIt Launcher"
echo "========================================="

# Clean up Snap environment variables that interfere with ROS2/RViz
# VS Code: Snap sets GTK/GIO paths that cause symbol lookup errors in rviz2
echo "[INFO] Cleaning Snap environment variables..."
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR

# Remove any snap library paths from LD_LIBRARY_PATH
if [[ -n "$LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
fi

# Kill any stale Gazebo/ROS processes from previous runs
echo "[INFO] Cleaning up stale processes..."
pkill -f "ign gazebo\|gz sim\|rviz2\|parameter_bridge\|robot_state_publisher\|move_group\|controller_manager\|spawner" 2>/dev/null || true
sleep 2

# Source ROS environment
echo "[INFO] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "[WARN] Workspace not built. Building spraying_pathways package..."
    cd "$SCRIPT_DIR"
    colcon build --packages-select spraying_pathways --symlink-install
else
    # Check if source files are newer than install (rebuild if modified)
    LAUNCH_SRC="$SCRIPT_DIR/src/spraying_pathways/launch/bringup_mycobot_320_gz_sim.launch.py"
    SRDF_SRC="$SCRIPT_DIR/src/spraying_pathways/config/mycobot_320_ur/mycobot_320_ur.srdf"
    INSTALL_MARKER="$SCRIPT_DIR/install/spraying_pathways/share/spraying_pathways/launch/bringup_mycobot_320_gz_sim.launch.py"
    
    if [ -f "$LAUNCH_SRC" ] && [ -f "$INSTALL_MARKER" ]; then
        if [ "$LAUNCH_SRC" -nt "$INSTALL_MARKER" ] || [ "$SRDF_SRC" -nt "$INSTALL_MARKER" ]; then
            echo "[INFO] Source files modified. Rebuilding spraying_pathways..."
            cd "$SCRIPT_DIR"
            colcon build --packages-select spraying_pathways --symlink-install
        fi
    fi
fi

# Source workspace
echo "[INFO] Sourcing workspace..."
cd "$SCRIPT_DIR"
source install/setup.bash

# Clear MoveIt warehouse database to avoid column errors
echo "[INFO] Clearing MoveIt warehouse database..."
rm -f ~/.ros/warehouse_ros.sqlite

# Launch the simulation
echo "[INFO] Launching myCobot 320 M5 with Gazebo Sim and MoveIt..."
echo "[INFO] Startup sequence (~25s total):"
echo "       1. Gazebo Sim starts (8s)"
echo "       2. Robot spawns (8s delay)"
echo "       3. Controllers activate: JSB → arm_controller (sequential)"
echo "       4. MoveIt move_group starts"
echo "       5. RViz opens with motion planning panel"
echo ""
echo "[INFO] To verify system is working:"
echo "       - Check Gazebo shows myCobot 320 robot"
echo "       - Check RViz shows robot model"
echo "       - In RViz Motion Planning: Plan and Execute should work"
echo "       - Test: ros2 control list_controllers (both should be 'active')"
echo "========================================="
echo ""

ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py

echo ""
echo "========================================="
echo "Simulation terminated"
echo "========================================="
