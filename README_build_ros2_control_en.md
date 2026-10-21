# Build ros2_control from source for ROS Jazzy

## Overview

Since the current ROS Jazzy version is still evolving, ros2_control and its related ROS packages are continuously being adapted and updated. If you encounter issues such as `controller_manager` or `hardware_interface` failing to load the URDF, you need to build and install the ros2_control related libraries from source.

### Remove currently installed ros2_control related packages

```shell
sudo apt remove \
ros-${ROS_DISTRO}-ros2-control \
ros-${ROS_DISTRO}-controller-manager \
ros-${ROS_DISTRO}-controller-manager-msgs \
ros-${ROS_DISTRO}-hardware-interface \
ros-${ROS_DISTRO}-ros2-controllers
ros-${ROS_DISTRO}-gz-ros2-control \
```

### Install dependencies for building
```shell
sudo apt install \
ros-${ROS_DISTRO}-ros2-control-cmake \
ros-${ROS_DISTRO}-pal-statistics \
ros-${ROS_DISTRO}-diagnostic-updater \
ros-${ROS_DISTRO}-control-toolbox \
ros-${ROS_DISTRO}-kinematics-interface \
ros-${ROS_DISTRO}-kinematics-interface-kdl \
ros-${ROS_DISTRO}-ackermann-msgs \
ros-${ROS_DISTRO}-tf2
```

### Download source code and build

```shell
cd ~/ros2_ws/src

# Download source code
git clone -b jazzy https://github.com/ros-controls/ros2_control.git
git clone -b jazzy https://github.com/ros-controls/ros2_controllers.git
git clone -b jazzy https://github.com/ros-controls/gz_ros2_control.git

cd ~/ros2_ws && colcon build

# Update environment variables to use the newly built packages
source install/setup.bash

# Verify installation
ros2 launch dexhand_bringup gz_position_controller.launch.py
```
### Debug

- If you find that Gazebo does not seem to load the gz_ros2_control plugin, try uninstalling Gazebo and reinstalling it.
```shell
sudo apt remove ros-${ROS_DISTRO}-ros-gz* ros-${ROS_DISTRO}-gz-*

sudo apt update && sudo apt install ros-${ROS_DISTRO}-ros-gz
```
