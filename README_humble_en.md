# ros2_botyard

## Overview

This document mainly describes some adjustments and adaptations for using ros2_botyard to control the dexterous hand in ROS Humble.

## Simulation Environment: Gazebo

Refer to the [Gazebo official website](https://gazebosim.org/docs/latest/ros_installation/) for recommendations and select the version that matches your ROS version.

For ROS Humble, the default compatible Gazebo version is Fortress. If you want to use Gazebo Harmonic, you can install it with the following commands:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get install \
ros-${ROS_DISTRO}-ros-gzharmonic \
ros-${ROS_DISTRO}-ros-gzharmonic-sim-demos \
ros-${ROS_DISTRO}-ros-gzharmonic-bridge \
ros-${ROS_DISTRO}-ros-gzharmonic-image \
ros-${ROS_DISTRO}-ros-gzharmonic-interfaces \
ros-${ROS_DISTRO}-ros-gzharmonic-sim
```

## Add Environment Variables

Open the .bashrc file
```bash
vim ~/.bashrc
```

Add the following at the end:
```bash
# gazebo use GPU
export LIBGL_ALWAYS_SOFTWARE=0
export GZ_VERSION=fortress
# Let gazebo find the mesh files for models
export GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share/:~/ros2_ws/install/dexhand_description/share/:~/ros2_ws/install/ur_description/share/
```
