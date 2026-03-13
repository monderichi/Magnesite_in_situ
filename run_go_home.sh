#!/bin/bash
# Run go_home_node with proper MoveIt parameters from running simulation
# Usage: ./run_go_home.sh (simulation must be running)

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

# Get robot_description and robot_description_semantic from move_group
ROBOT_DESC=$(ros2 param get /move_group robot_description 2>/dev/null | tail -n +2)
ROBOT_SEMANTIC=$(ros2 param get /move_group robot_description_semantic 2>/dev/null | tail -n +2)

if [ -z "$ROBOT_DESC" ]; then
    echo "[ERROR] move_group not running or robot_description not available"
    echo "[INFO] Start simulation first: ./launch_mycobot320_sim.sh"
    exit 1
fi

echo "[INFO] Running go_home_node with MoveIt parameters..."
ros2 run spraying_pathways go_home_node \
    --ros-args \
    -p robot_description:="$ROBOT_DESC" \
    -p robot_description_semantic:="$ROBOT_SEMANTIC"
