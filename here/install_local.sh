#!/bin/bash

# This script installs ROS 2 Humble and all project dependencies locally on Ubuntu 22.04.
# It mimics the setup found in the project's Dockerfile.

set -e

echo "Starting local installation of ROS 2 Humble and dependencies..."

# 1. Ensure UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add ROS 2 Humble repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Update and install base ROS 2 and build tools
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    python3-pip

# 4. Install specific project dependencies (from Dockerfile)
sudo apt install -y \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-ur-description \
    ros-humble-tf-transformations \
    gazebo \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-ur-moveit-config \
    libyaml-cpp-dev

# 5. Install Python dependencies
sudo apt install -y python3-numpy python3-sklearn
pip3 install --user numpy-stl

# 6. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 7. Install additional dependencies via rosdep from workspace
echo "Installing workspace dependencies via rosdep..."
export ROS_DISTRO=humble
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws
rosdep install --from-paths src --ignore-src -y -r --rosdistro humble

echo ""
echo "===================================================="
echo "Installation complete!"
echo "To build your workspace, run:"
echo "  cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "Then you can run:"
echo "  ros2 launch spraying_pathways bringup_v4.launch.py"
echo "===================================================="
