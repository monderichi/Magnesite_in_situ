#!/bin/bash
# Standalone launcher for myCobot 320 M5 + RealSense D455
# Camera is on a separate stand: 20cm X offset, 30cm above work position
# 
# Usage: ./launch_mycobot320_realsense.sh [options]
# 
# Options:
#   --real          Run with REAL robot (no Gazebo)
#   --sim           Run with Gazebo simulation (default)
#   --no-camera     Don't launch RealSense camera
#   --port PORT     Serial port for real robot (default: /dev/ttyACM0)
#   --x <value>     Camera X position (default: 0.5)
#   --y <value>     Camera Y position (default: 0.0)
#   --z <value>     Camera Z position (default: 0.5)
#
# Examples:
#   ./launch_mycobot320_realsense.sh                    # Simulation + camera
#   ./launch_mycobot320_realsense.sh --real             # Real robot + camera
#   ./launch_mycobot320_realsense.sh --sim --no-camera  # Simulation only
#   ./launch_mycobot320_realsense.sh --real --port /dev/ttyUSB0 --x 0.6

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default values
USE_GAZEBO="true"
RUN_REALSENSE="true"
ROBOT_PORT="/dev/ttyACM0"
CAMERA_X="-0.245"
CAMERA_Y="0.255"
CAMERA_Z="0.552"
CAMERA_ROLL="0.0"
CAMERA_PITCH="1.5708"
CAMERA_YAW="0.0"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --real)
            USE_GAZEBO="false"
            shift
            ;;
        --sim)
            USE_GAZEBO="true"
            shift
            ;;
        --no-camera)
            RUN_REALSENSE="false"
            shift
            ;;
        --port)
            ROBOT_PORT="$2"
            shift 2
            ;;
        --x)
            CAMERA_X="$2"
            shift 2
            ;;
        --y)
            CAMERA_Y="$2"
            shift 2
            ;;
        --z)
            CAMERA_Z="$2"
            shift 2
            ;;
        --roll)
            CAMERA_ROLL="$2"
            shift 2
            ;;
        --pitch)
            CAMERA_PITCH="$2"
            shift 2
            ;;
        --yaw)
            CAMERA_YAW="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --real          Run with REAL robot (no Gazebo simulation)"
            echo "  --sim           Run with Gazebo simulation (default)"
            echo "  --no-camera     Don't launch RealSense camera"
            echo "  --port PORT     Serial port for real robot (default: /dev/ttyACM0)"
            echo "  --x <value>     Camera X position (default: 0.5)"
            echo "  --y <value>     Camera Y position (default: 0.0)"
            echo "  --z <value>     Camera Z position (default: 0.5)"
            echo "  --roll <value>  Camera roll (default: 0.0)"
            echo "  --pitch <value> Camera pitch (default: 1.5708 for 90deg top-down)"
            echo "  --yaw <value>   Camera yaw (default: 0.0)"
            echo "  -h, --help      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                              # Simulation + camera"
            echo "  $0 --real                       # Real robot + camera"
            echo "  $0 --sim --no-camera            # Simulation only"
            echo "  $0 --real --port /dev/ttyUSB0   # Real robot, custom port"
            echo "  $0 --real --x 0.6 --z 0.55      # Real robot, custom camera pos"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo "========================================="
echo "myCobot 320 M5 + RealSense D455 Launcher"
echo "========================================="
echo ""

if [ "$USE_GAZEBO" = "true" ]; then
    echo "Mode: SIMULATION (Gazebo)"
else
    echo "Mode: REAL ROBOT"
    echo "Serial Port: $ROBOT_PORT"
fi

echo "RealSense: $([ "$RUN_REALSENSE" = "true" ] && echo "ENABLED" || echo "DISABLED")"

if [ "$RUN_REALSENSE" = "true" ]; then
    echo ""
    echo "Camera position (relative to base_link):"
    echo "  X: ${CAMERA_X}m (forward from base)"
    echo "  Y: ${CAMERA_Y}m (left/right from base)"
    echo "  Z: ${CAMERA_Z}m (height above base)"
    echo "  Default orientation: looking DOWN (top-down view)"
fi

echo ""
echo "========================================="

# Real robot safety check
if [ "$USE_GAZEBO" = "false" ]; then
    echo ""
    echo "⚠️  SAFETY CHECKLIST FOR REAL ROBOT:"
    echo "   1. Ensure robot has CLEAR WORKSPACE"
    echo "   2. Emergency stop button is ACCESSIBLE"
    echo "   3. Robot is powered ON"
    echo "   4. USB cable is connected"
    echo ""
    read -p "Have you completed the safety checklist? (yes/no): " safety_check
    
    if [[ "$safety_check" != "yes" ]]; then
        echo "Please complete the safety checklist before proceeding."
        exit 1
    fi
    
    # Check if port exists
    if [ ! -e "$ROBOT_PORT" ]; then
        echo ""
        echo "❌ ERROR: Serial port $ROBOT_PORT does not exist!"
        echo "Available ports:"
        ls -la /dev/tty* 2>/dev/null | grep -E "ttyACM|ttyUSB" || echo "   No /dev/ttyACM* or /dev/ttyUSB* ports found"
        echo ""
        echo "Troubleshooting:"
        echo "   1. Check USB connection"
        echo "   2. Run: ls /dev/tty* to find the correct port"
        echo "   3. Try: sudo chmod 666 $ROBOT_PORT"
        exit 1
    fi
    
    echo "✓ Serial port $ROBOT_PORT found"
    
    # Check port permissions
    if [ ! -r "$ROBOT_PORT" ] || [ ! -w "$ROBOT_PORT" ]; then
        echo "⚠️  Fixing permissions on $ROBOT_PORT..."
        sudo chmod 666 "$ROBOT_PORT" 2>/dev/null || {
            echo "❌ Failed to fix permissions. Try:"
            echo "   sudo usermod -a -G dialout \$USER"
            echo "   Then log out and log back in."
            exit 1
        }
        echo "✓ Permissions fixed"
    fi
fi

# Clean up Snap environment variables
echo ""
echo "[INFO] Cleaning environment..."
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GTK_MODULES GTK2_MODULES GTK3_MODULES
unset GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_PLATFORMTHEME

if [[ -n "$LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
fi

export QT_QPA_PLATFORM=xcb

# Kill stale processes
echo "[INFO] Cleaning up stale processes..."
pkill -f "ign gazebo\|gz sim\|rviz2\|parameter_bridge\|robot_state_publisher\|move_group\|controller_manager\|spawner\|realsense2\|static_transform\|mycobot_driver" 2>/dev/null || true
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
    # Check if source files are newer than install
    LAUNCH_SRC="$SCRIPT_DIR/src/spraying_pathways/launch/bringup_mycobot_320_realsense.launch.py"
    INSTALL_MARKER="$SCRIPT_DIR/install/spraying_pathways/share/spraying_pathways/launch/bringup_mycobot_320_realsense.launch.py"
    
    if [ -f "$LAUNCH_SRC" ] && [ -f "$INSTALL_MARKER" ]; then
        if [ "$LAUNCH_SRC" -nt "$INSTALL_MARKER" ]; then
            echo "[INFO] Launch file modified. Rebuilding..."
            cd "$SCRIPT_DIR"
            colcon build --packages-select spraying_pathways --symlink-install
        fi
    fi
fi

# Source workspace
echo "[INFO] Sourcing workspace..."
cd "$SCRIPT_DIR"
source install/setup.bash

# Clear MoveIt warehouse database
echo "[INFO] Clearing MoveIt warehouse database..."
rm -f ~/.ros/warehouse_ros.sqlite

# Check if RealSense camera is connected (if needed)
if [ "$RUN_REALSENSE" = "true" ]; then
    if lsusb | grep -q "Intel.*RealSense"; then
        echo "[INFO] RealSense camera detected via USB"
        lsusb | grep "Intel.*RealSense"
        
        # Check USB power management and disable autosuspend for RealSense
        echo "[INFO] Configuring USB power management for RealSense..."
        for device in /sys/bus/usb/devices/*/idVendor; do
            if [ -f "$device" ] && grep -q "8086" "$device" 2>/dev/null; then
                usb_path=$(dirname "$device")
                echo "on" > "$usb_path/power/control" 2>/dev/null || true
                echo "[INFO] Disabled USB autosuspend for RealSense at $(basename $usb_path)"
            fi
        done
        
        # Set RealSense udev rules if not already set
        if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
            echo "[WARN] RealSense udev rules may not be installed."
            echo "       Run: sudo apt install ros-humble-realsense2-camera"
        fi
        
        # Check USB port power
        echo "[INFO] USB RealSense device info:"
        lsusb -v -d 8086: 2>/dev/null | grep -E "(Bus|Device|MaxPower|bcdUSB)" | head -10 || true
        
    else
        echo "[WARN] No RealSense camera detected!"
        echo "       Continuing without camera..."
        RUN_REALSENSE="false"
    fi
fi

# Launch
echo ""
echo "========================================="
echo "Launching..."
echo "========================================="

if [ "$USE_GAZEBO" = "true" ]; then
    echo "[INFO] Startup sequence (~25s total):"
    echo "       1. Gazebo Sim starts (8s)"
    echo "       2. Robot spawns (8s delay)"
    echo "       3. Controllers activate: JSB -> arm_controller"
    echo "       4. MoveIt move_group starts"
    echo "       5. RViz opens with motion planning panel"
    echo "       6. RealSense camera starts (15s delay)"
else
    echo "[INFO] Startup sequence (~30s total):"
    echo "       1. Robot driver starts (connects to hardware)"
    echo "       2. MoveIt move_group starts (5s delay)"
    echo "       3. RViz opens with motion planning panel"
    echo "       4. RealSense camera starts (20s delay)"
fi

echo ""
echo "[INFO] To control the robot:"
if [ "$USE_GAZEBO" = "false" ]; then
    echo "       ros2 run spraying_pathways go_home_node"
    echo "       ros2 run spraying_pathways cartesian_path_planner_trajectory_v1_mycobot320_node"
fi
echo ""
echo "[INFO] To monitor joint states:"
echo "       ros2 topic echo /joint_states"
echo ""
echo "========================================="
echo ""

# Run the launch file
ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py \
    use_gazebo:="${USE_GAZEBO}" \
    robot_port:="${ROBOT_PORT}" \
    camera_x:="${CAMERA_X}" \
    camera_y:="${CAMERA_Y}" \
    camera_z:="${CAMERA_Z}" \
    camera_roll:="${CAMERA_ROLL}" \
    camera_pitch:="${CAMERA_PITCH}" \
    camera_yaw:="${CAMERA_YAW}" \
    run_realsense:="${RUN_REALSENSE}"

echo ""
echo "========================================="
echo "Terminated"
echo "========================================="
