#!/bin/bash
# Launch script for REAL myCobot 320 M5 robot
# This script sets up the environment and launches MoveIt with the real robot
# NO simulation, NO RViz - pure hardware control

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default serial port
PORT="/dev/ttyACM0"
BAUD="115200"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --port)
            PORT="$2"
            shift 2
            ;;
        --baud)
            BAUD="$2"
            shift 2
            ;;
        --help)
            echo "Usage: ./launch_real_robot.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --port PORT    Serial port (default: /dev/ttyACM0)"
            echo "  --baud BAUD    Baud rate (default: 115200)"
            echo "  --help         Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./launch_real_robot.sh"
            echo "  ./launch_real_robot.sh --port /dev/ttyUSB0"
            echo "  ./launch_real_robot.sh --port /dev/ttyACM0 --baud 115200"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "========================================="
echo "myCobot 320 M5 REAL ROBOT Launcher"
echo "========================================="
echo "Serial Port: $PORT"
echo "Baud Rate: $BAUD"
echo ""

# Safety warnings
echo "⚠️  SAFETY CHECKLIST:"
echo "   1. Ensure robot has CLEAR WORKSPACE"
echo "   2. Emergency stop button is ACCESSIBLE"
echo "   3. All cables are connected properly"
echo "   4. Robot is powered ON"
echo ""
read -p "Have you completed the safety checklist? (yes/no): " safety_check

if [[ "$safety_check" != "yes" ]]; then
    echo "Please complete the safety checklist before proceeding."
    exit 1
fi

echo ""

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "❌ ERROR: Serial port $PORT does not exist!"
    echo "Available ports:"
    ls -la /dev/tty* 2>/dev/null | grep -E "ttyACM|ttyUSB" || echo "   No /dev/ttyACM* or /dev/ttyUSB* ports found"
    echo ""
    echo "Troubleshooting:"
    echo "   1. Check USB connection"
    echo "   2. Run: ls /dev/tty* to find the correct port"
    echo "   3. Try: sudo chmod 666 $PORT"
    exit 1
fi

echo "✓ Serial port $PORT found"

# Check port permissions
if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "⚠️  WARNING: Permission issue with $PORT"
    echo "Fixing permissions..."
    sudo chmod 666 "$PORT" 2>/dev/null || {
        echo "❌ Failed to fix permissions. Try:"
        echo "   sudo usermod -a -G dialout \$USER"
        echo "   Then log out and log back in."
        exit 1
    }
    echo "✓ Permissions fixed"
fi

# Source ROS environment
echo ""
echo "[INFO] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "[WARN] Workspace not built. Building..."
    cd "$SCRIPT_DIR"
    colcon build --symlink-install
fi

# Source workspace
echo "[INFO] Sourcing workspace..."
cd "$SCRIPT_DIR"
source install/setup.bash

# Check if pymycobot is installed
echo "[INFO] Checking pymycobot installation..."
if ! python3 -c "from pymycobot import MyCobot320" 2>/dev/null; then
    echo "⚠️  WARNING: pymycobot not installed!"
    echo "Install with: pip3 install pymycobot"
    echo ""
    echo "The driver will run in MOCK MODE (no real robot communication)"
    echo ""
    read -p "Continue in MOCK MODE? (yes/no): " mock_continue
    if [[ "$mock_continue" != "yes" ]]; then
        exit 1
    fi
fi

echo ""
echo "========================================="
echo "Starting REAL ROBOT control..."
echo ""
echo "Components:"
echo "  - Robot State Publisher"
echo "  - MyCobot Driver (port: $PORT)"
echo "  - MoveIt Move Group"
echo ""
echo "Nodes started:"
echo "  ✓ /robot_state_publisher"
echo "  ✓ /mycobot_driver"
echo "  ✓ /move_group"
echo ""
echo "To control the robot:"
echo "  ros2 run spraying_pathways go_home_node"
echo "  ros2 run spraying_pathways cartesian_path_planner_trajectory_v1_mycobot320_node"
echo ""
echo "To monitor joint states:"
echo "  ros2 topic echo /joint_states"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Launch the real robot
ros2 launch spraying_pathways real_robot.launch.py port:=$PORT baud:=$BAUD

echo ""
echo "========================================="
echo "Real robot control terminated"
echo "========================================="
