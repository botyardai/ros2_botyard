# ros2_botyard

## Overview

The workspace `ros2_botyard` is mainly used to manage ROS2 packages related to robotic hand control. Among them, the `dexhand_bringup` package contains configuration files for launching and configuring the robotic hand controllers.

Directory Structure

```bash
.
└── ros2_botyard
    ├── dexhand_bringup
    │   ├── config
    │   ├── launch
    │   └── test
    │
    ├── dexhand_control
    │   ├── hardwares
    │   │   └── dexhand_hardware_interface
    │   └── controllers
    │       └── dexhand_controller
    │
    └── dexhand_description
        ├── config
        ├── launch
        ├── meshes
        ├── rviz
        └── urdf
```

## Environment Setup

### Install ros2 Jazzy (ubuntu24.04)

Set locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Install repositories

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
You can check the IP address of githubusercontent.com at [ipaddress](https://www.ipaddress.com/website/raw.githubusercontent.com/) and use the command line to change hosts
```bash
sudo tee -a /etc/hosts <<EOF
185.199.108.133 raw.githubusercontent.com
EOF
```

Add the repository to your sources list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ros2 Desktop ROS-Base

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-base ros-dev-tools
```

Add this command to your `~/.bashrc` file

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Dependency Installation

Control framework ros2-control
We also use PickNikRobotics's [parameter library generation tool](https://github.com/PickNikRobotics/generate_parameter_library) to assist in generating code for reading parameters.

```bash
sudo apt-get install \
ros-${ROS_DISTRO}-ros2-control \
ros-${ROS_DISTRO}-ros2-controllers \
ros-${ROS_DISTRO}-generate-parameter-library
```

Simulation environment Gazebo
Refer to the [Gazebo official website](https://gazebosim.org/docs/latest/ros_installation/) for recommendations and select the version that matches your ROS version.

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Select the version matching ROS, here is jazzy
export GZ_VERSION=harmonic

sudo apt-get install \
ros-${ROS_DISTRO}-ros-gz \
ros-${ROS_DISTRO}-ros-gz-sim \
ros-${ROS_DISTRO}-ros-gz-bridge \
ros-${ROS_DISTRO}-ros-gz-image \
ros-${ROS_DISTRO}-ros-gz-interfaces \
ros-${ROS_DISTRO}-ros-gz-sim-demos \
# Control the robot in Gazebo via ros2_control framework
ros-${ROS_DISTRO}-gz-ros2-control
```

Motion planning MoveIt

```bash
sudo apt-get install \
ros-${ROS_DISTRO}-moveit \
ros-${ROS_DISTRO}-moveit-planners-chomp \
ros-${ROS_DISTRO}-moveit-visual-tools \
ros-${ROS_DISTRO}-rviz-visual-tools \
ros-${ROS_DISTRO}-pick-ik
```

## Source Code Compilation

First, please read the [official ROS documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and create your workspace.

```bash
# Clone the repository into your ROS workspace src/ directory
cd ~/ros2_ws/src
git clone git@gitee.com:Botyard/ros2_botyard.git

# install the driver in the repository, dexhand_driver
sudo apt install ~/ros2_ws/src/ros2_botyard/debs/dexhand-driver.deb

# Go to the workspace directory and build the downloaded packages
cd ~/ros2_ws/ && colcon build

# For the first build, source the workspace before use
source install/setup.sh
```

### Add Environment Variables

Open the .bashrc file

```bash
vim ~/.bashrc
```

Add the following at the end

```bash
# ros jazzy
source /opt/ros/jazzy/setup.bash
# pkgs build from source
source ~/ros2_ws/install/setup.bash

# for rosdep see: https://mirrors4.tuna.tsinghua.edu.cn/help/rosdistro/
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml

# gazebo use GPU
export LIBGL_ALWAYS_SOFTWARE=0
export GZ_VERSION=harmonic
# Let gazebo find the mesh files for models
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/:~/ros2_ws/install/dexhand_description/share/:~/ros2_ws/install/ur_description/share/
```

Update environment variables or restart the terminal

```bash
source ~/.bashrc
```

## Add udev Rules

To avoid having to confirm the device /dev/ttyXXX and permission issues every time you plug in the dexterous hand, you can add `dexhand_description/config/99-botyard.rules` to your system.

```bash
sudo cp dexhand_description/config/99-botyard.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Get sdf Files

```bash
# Convert xcaro file to urdf
xacro dexhand.urdf.xacro > dexhand.urdf
# Export right hand model
xacro use_left_hand:=false ./dexhand.urdf.xacro > dexhand_rh.urdf

# Convert urdf file to sdf
gz sdf -p dexhand.urdf > dexhand.sdf
```

## Run

### rviz Display

Download joint state publisher
```bash
sudo apt-get install ros-${ROS_DISTRO}-joint-state-*
```
Start rviz
```bash
ros2 launch dexhand_description play.launch.py
```

### Gazebo Simulation

```bash
# dexhand model
ros2 launch dexhand_description gazebo.launch.py

# Integrated dual hand model
ros2 launch dexhand_description gazebo.launch.py urdf_path:=urdf/two_dexhand_as_whole.urdf.xacro use_rviz:=true
```
