#!/bin/bash
# Diagnostic script for myCobot 320 M5 MoveIt2 + Gazebo Sim setup
# Tests each component to verify fixes

set -e

echo "========================================="
echo "myCobot 320 M5 MoveIt Setup Diagnostics"
echo "========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS=0
FAIL=0

# Function to check if a package is installed
check_package() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} Package $1 is installed"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗${NC} Package $1 is NOT installed"
        ((FAIL++))
        return 1
    fi
}

# Function to check if a file exists
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} File exists: $1"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗${NC} File missing: $1"
        ((FAIL++))
        return 1
    fi
}

echo ""
echo "1. Checking Required Packages..."
echo "-----------------------------------"
check_package "mycobot_description"
check_package "spraying_pathways"
check_package "ros_gz_sim"
check_package "gz_ros2_control"
check_package "moveit_ros_move_group"

echo ""
echo "2. Checking Configuration Files..."
echo "-----------------------------------"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ros2_controllers.yaml"
check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/moveit_controllers.yaml"
check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/kinematics.yaml"
check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/joint_limits.yaml"
check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ompl_planning.yaml"
check_file "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/mycobot_320_ur.srdf"

echo ""
echo "3. Checking URDF/Xacro Files..."
echo "-----------------------------------"
check_file "$WS_DIR/src/mycobot_description/urdf/robots/mycobot_320_ur_compatible.urdf.xacro"
check_file "$WS_DIR/src/mycobot_description/urdf/control/mycobot_320_ur_ros2_control.urdf.xacro"
check_file "$WS_DIR/src/mycobot_description/urdf/control/gazebo_sim_ros2_control.urdf.xacro"

echo ""
echo "4. Validating Xacro Processing..."
echo "-----------------------------------"

# Test xacro processing
cd "$WS_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || true

XACRO_OUTPUT=$(xacro "$WS_DIR/src/mycobot_description/urdf/robots/mycobot_320_ur_compatible.urdf.xacro" \
    robot_name:=mycobot_320 \
    use_gazebo:=true \
    controllers_file:=none 2>&1) && {
    echo -e "${GREEN}✓${NC} Xacro processing succeeded"
    ((PASS++))
    
    # Check for critical elements in output
    if echo "$XACRO_OUTPUT" | grep -q "gz_ros2_control/GazeboSimSystem"; then
        echo -e "${GREEN}✓${NC} Gazebo Sim plugin is correctly set"
        ((PASS++))
    else
        echo -e "${RED}✗${NC} Gazebo Sim plugin not found in URDF"
        ((FAIL++))
    fi
    
    if echo "$XACRO_OUTPUT" | grep -q "shoulder_pan_joint"; then
        echo -e "${GREEN}✓${NC} UR-compatible joint names found"
        ((PASS++))
    else
        echo -e "${RED}✗${NC} UR-compatible joint names not found"
        ((FAIL++))
    fi
    
    if echo "$XACRO_OUTPUT" | grep -q "libgz_ros2_control-system.so"; then
        echo -e "${GREEN}✓${NC} Correct plugin library (libgz_ros2_control-system.so)"
        ((PASS++))
    else
        echo -e "${RED}✗${NC} Plugin library not correct"
        ((FAIL++))
    fi
} || {
    echo -e "${RED}✗${NC} Xacro processing failed:"
    echo "$XACRO_OUTPUT"
    ((FAIL++))
}

echo ""
echo "5. Checking Controller Configuration..."
echo "-----------------------------------"

# Validate YAML files
if python3 -c "import yaml; yaml.safe_load(open('$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ros2_controllers.yaml'))" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} ros2_controllers.yaml is valid YAML"
    ((PASS++))
else
    echo -e "${RED}✗${NC} ros2_controllers.yaml has YAML errors"
    ((FAIL++))
fi

if python3 -c "import yaml; yaml.safe_load(open('$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ompl_planning.yaml'))" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} ompl_planning.yaml is valid YAML"
    ((PASS++))
else
    echo -e "${RED}✗${NC} ompl_planning.yaml has YAML errors"
    ((FAIL++))
fi

# Check for duplicate planner_configs key
if grep -c "^planner_configs:" "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ompl_planning.yaml" | grep -q "^1$"; then
    echo -e "${GREEN}✓${NC} ompl_planning.yaml has no duplicate keys"
    ((PASS++))
else
    echo -e "${RED}✗${NC} ompl_planning.yaml has duplicate planner_configs keys"
    ((FAIL++))
fi

# Check joint names match between SRDF and controllers
SRDF_JOINTS=$(grep -oP 'joint name="\K[^"]+' "$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/mycobot_320_ur.srdf" | sort -u)
CONTROLLER_JOINTS=$(python3 -c "import yaml; d=yaml.safe_load(open('$WS_DIR/src/spraying_pathways/config/mycobot_320_ur/ros2_controllers.yaml')); print('\n'.join(d.get('arm_controller', {}).get('ros__parameters', {}).get('joints', [])))" 2>/dev/null | sort -u)

if [ "$SRDF_JOINTS" = "$CONTROLLER_JOINTS" ]; then
    echo -e "${GREEN}✓${NC} Joint names match between SRDF and controllers"
    ((PASS++))
else
    echo -e "${YELLOW}⚠${NC} Joint names may differ between SRDF and controllers"
    echo "  SRDF joints: $SRDF_JOINTS"
    echo "  Controller joints: $CONTROLLER_JOINTS"
fi

echo ""
echo "6. Checking Launch File..."
echo "-----------------------------------"
check_file "$WS_DIR/src/spraying_pathways/launch/bringup_mycobot_320_gz_sim.launch.py"

# Check for OnProcessStart instead of OnProcessExit
if grep -q "OnProcessStart" "$WS_DIR/src/spraying_pathways/launch/bringup_mycobot_320_gz_sim.launch.py"; then
    echo -e "${GREEN}✓${NC} Launch file uses OnProcessStart (correct for persistent nodes)"
    ((PASS++))
else
    echo -e "${RED}✗${NC} Launch file may have timing issues"
    ((FAIL++))
fi

echo ""
echo "========================================="
echo "Summary: $PASS passed, $FAIL failed"
echo "========================================="

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}All checks passed! You should be ready to launch.${NC}"
    echo ""
    echo "To launch the simulation, run:"
    echo "  cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws"
    echo "  ./launch_mycobot320_sim.sh"
    exit 0
else
    echo -e "${YELLOW}Some checks failed. Please review the errors above.${NC}"
    exit 1
fi
