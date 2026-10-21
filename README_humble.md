# ros2_botyard

## 概述

这里主要针对在ROS Humble版本中使用ros2_botyard控制灵巧手时的一些调整和适配。


## 仿真环境Gazebo

参考[Gazebo官网](https://gazebosim.org/docs/latest/ros_installation/)建议，选择跟ROS版本匹配的。

对于ROS Humble版本，默认适配的Gazebo版本为Fortress。如果想要使用Gazebo Harmonic版本，可以用以下命令进行安装:

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

## 添加环境变量

打开.bashrc文件
```bash
vim ~/.bashrc
```

在最后添加下面内容
```bash
# gazebo use GPU
export LIBGL_ALWAYS_SOFTWARE=0
export GZ_VERSION=fortress
# 让gazebo能够找到模型的mesh文件
export GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share/:~/ros2_ws/install/dexhand_description/share/:~/ros2_ws/install/ur_description/share/
```

sudo apt install ros-jazzy-pal-statistics
