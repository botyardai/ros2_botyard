# ros2_botyard

## 概述

本工作空间 `ros2_botyard` 主要用于管理与机器人手部控制相关的 ROS2 包。其中，`dexhand_bringup` 包包含了用于启动和配置机器人手部控制器的配置文件。

目录结构

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

## 环境搭建

### 安装 ros2 Jazzy(ubuntu24.04)

Set locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

install repositories

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

add the ROS 2 GPG key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
可以在[ipaddress](https://www.ipaddress.com/website/raw.githubusercontent.com/)查看githubusercontent.com的ip地址，并使用命令行更改hosts
```bash
sudo tee -a /etc/hosts <<EOF
185.199.108.133 raw.githubusercontent.com
EOF
```

add the repository to your sources list

````bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
````

Install ros2 Desktop ROS-Base

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-base ros-dev-tools
```

将此命令添加到 `~/.bashrc` 文件中

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 依赖安装

#### 安装仓库中自带的灵巧手驱动 dexhand_driver

```bash
## ROS Jazzy(Ubuntu 24.04)
sudo apt install ~/ros2_ws/src/ros2_botyard/debs/dexhand-driver.deb

## ROS Humble(Ubuntu 22.04)
sudo apt install ~/ros2_ws/src/ros2_botyard/debs/dexhand-driver_humble.deb

```

#### 控制框架ros2-control
同时我们使用PickNikRobotic的[参数校验代码生成工具](https://github.com/PickNikRobotics/generate_parameter_library)辅助生成读取Paramerter的代码。

```bash
sudo apt-get install \
ros-${ROS_DISTRO}-ros2-control \
ros-${ROS_DISTRO}-ros2-controllers \
ros-${ROS_DISTRO}-generate-parameter-library \
ros-${ROS_DISTRO}-pal-statistics
```

#### 仿真环境Gazebo
参考[Gazebo官网](https://gazebosim.org/docs/latest/ros_installation/)建议，选择跟ROS版本匹配的。

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 选择跟ROS版本匹配的，这里是jazzy
export GZ_VERSION=harmonic

sudo apt-get install \
ros-${ROS_DISTRO}-ros-gz \
ros-${ROS_DISTRO}-ros-gz-sim \
ros-${ROS_DISTRO}-ros-gz-bridge \
ros-${ROS_DISTRO}-ros-gz-image \
ros-${ROS_DISTRO}-ros-gz-interfaces \
ros-${ROS_DISTRO}-ros-gz-sim-demos \
# 通过ros2_control框架控制gazebo里的机器人
ros-${ROS_DISTRO}-gz-ros2-control
```

#### 运动规划MoveIt2

```bash
sudo apt-get install \
ros-${ROS_DISTRO}-moveit \
ros-${ROS_DISTRO}-moveit-planners-chomp \
ros-${ROS_DISTRO}-moveit-visual-tools \
ros-${ROS_DISTRO}-rviz-visual-tools \
ros-${ROS_DISTRO}-pick-ik
```

## 源码编译

首先请先阅读[ROS官方文档](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)，创建好工作空间。

```bash
# 将仓库clone到你的ROS工作空间src/目录下
cd ~/ros2_ws/src
git clone git@gitee.com:Botyard/ros2_botyard.git

# 跳转到工作空间目录编译下载的pkg
cd ~/ros2_ws/ && colcon build

# 首次编译的pkg需要source一下才能使用
source install/setup.sh

```

### 添加环境变量

打开.bashrc文件

```bash
vim ~/.bashrc
```

在最后添加下面内容

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
# 让gazebo能够找到模型的mesh文件
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/:~/ros2_ws/install/dexhand_description/share/:~/ros2_ws/install/ur_description/share/
```

更新环境变量或者重开终端

```bash
source ~/.bashrc
```

## 添加udev规则

为避免每次插入灵巧手时都要确定设备/dev/ttyXXX和权限问题，可以将`dexhand_description/config/99-botyard.rules`添加到系统中。

```bash
sudo cp dexhand_description/config/99-botyard.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger

```

## 获取sdf文件

```bash
# xcaro文件转 urdf
xacro dexhand.urdf.xacro > dexhand.urdf
# 导出右手模型
xacro use_left_hand:=false ./dexhand.urdf.xacro > dexhand_rh.urdf

# urdf 文件转 sdf
gz sdf -p dexhand.urdf > dexhand.sdf
```

## 运行

### rviz显示

下载关节发布器
```bash
 sudo apt-get install ros-${ROS_DISTRO}-joint-state-*
```
启动rviz
```bash
ros2 launch dexhand_description play.launch.py
```

### Gazebo仿真

```bash
# dexhand模型
ros2 launch dexhand_description gazebo.launch.py

# 集成后的双手模型
ros2 launch dexhand_description gazebo.launch.py urdf_path:=urdf/two_dexhand_as_whole.urdf.xacro use_rviz:=true
```
