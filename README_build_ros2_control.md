# Build ros2_control from source for ROS Jazzy

## 概述

因为当前ROS的Jazzy版本还在不断的迭代，ros2_control及其相关联的ros包也都在不断的适配更新。
如果碰到关于`controller_manager`或者`hardware_interface`加载不了URDF的情况，我们需要从源代码编译安装ros2_control相关的库。

### 删除当前安装的ros2_control相关包

``` shell
sudo apt remove \
ros-${ROS_DISTRO}-ros2-control \
ros-${ROS_DISTRO}-controller-manager \
ros-${ROS_DISTRO}-controller-manager-msgs \
ros-${ROS_DISTRO}-hardware-interface \
ros-${ROS_DISTRO}-ros2-controllers
ros-${ROS_DISTRO}-gz-ros2-control \
```

### 安装编译的依赖库
``` shell
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

### 下载源码编译

``` shell
cd ~/ros2_ws/src

# 下载源码
git clone -b jazzy https://github.com/ros-controls/ros2_control.git
git clone -b jazzy https://github.com/ros-controls/ros2_controllers.git
git clone -b jazzy https://github.com/ros-controls/gz_ros2_control.git

cd ~/ros2_ws && colcon build

# 修改环境变量，使新编译的包生效
source install/setup.bash

# 验证是否安装成功
ros2 launch dexhand_bringup gz_position_controller.launch.py
```
### Debug

- 如果发现gazebo似乎没有加载gz_ros2_control的插件，可以尝试卸载掉gazebo并重新安装。
``` shell
sudo apt remove ros-${ROS_DISTRO}-ros-gz* ros-${ROS_DISTRO}-gz-*

sudo apt update && sudo apt install ros-${ROS_DISTRO}-ros-gz
```
