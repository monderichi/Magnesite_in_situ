#!/bin/bash
# Debug launcher for myCobot 320 M5 in Gazebo Sim with MoveIt
# Runs with verbose logging for debugging
# Usage: ./launch_mycobot320_sim_debug.sh

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create log directory
LOG_DIR="$SCRIPT_DIR/log"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/mycobot320_sim_${TIMESTAMP}.log"

echo "========================================="
echo "myCobot 320 M5 Gazebo Sim + MoveIt DEBUG Launcher"
echo "========================================="
echo "Log file: $LOG_FILE"
echo ""

# Clean up Snap environment variables that interfere with ROS2/RViz
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

# Build mycobot_description first
echo "[INFO] Building mycobot_description..."
cd "$SCRIPT_DIR"
colcon build --packages-select mycobot_description --cmake-clean-cache 2>&1 | tee -a "$LOG_FILE"

# Build spraying_pathways
echo "[INFO] Building spraying_pathways..."
colcon build --packages-select spraying_pathways 2>&1 | tee -a "$LOG_FILE"

# Source workspace
echo "[INFO] Sourcing workspace..."
source install/setup.bash

# Clear MoveIt warehouse database
echo "[INFO] Clearing MoveIt warehouse database..."
rm -f ~/.ros/warehouse_ros.sqlite

# Check xacro parsing
echo "[INFO] Testing xacro parsing..."
echo "--- Testing mycobot_320_ur_compatible.urdf.xacro ---" | tee -a "$LOG_FILE"
xacro "$SCRIPT_DIR/src/mycobot_description/urdf/robots/mycobot_320_ur_compatible.urdf.xacro" \
    robot_name:=mycobot_320 \
    prefix:="" \
    use_gazebo:=true \
    controllers_file:="$SCRIPT_DIR/src/spraying_pathways/config/mycobot_320_ur/ros2_controllers.yaml" \
    2>&1 | head -100 | tee -a "$LOG_FILE"

if [ $? -eq 0 ]; then
    echo "✓ Xacro parsing successful" | tee -a "$LOG_FILE"
else
    echo "✗ Xacro parsing failed!" | tee -a "$LOG_FILE"
    exit 1
fi

# Launch the simulation with verbose output
echo ""
echo "========================================="
echo "Launching myCobot 320 M5 with DEBUG logging..."
echo "========================================="
echo ""

# Set verbose logging
export RCUTILS_LOGGING_SEVERITY=DEBUG
export ROS_LOG_DIR="$LOG_DIR"

# Launch with output to both console and log file
ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py \
    gazebo_gui:=true \
    2>&1 | tee -a "$LOG_FILE"

echo ""
echo "========================================="
echo "Simulation terminated"
echo "Log saved to: $LOG_FILE"
echo "========================================="
