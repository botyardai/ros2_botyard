# ros2_botyard

## 概述

`ros2_botyard` 是用于灵巧手模型描述、ROS 2 硬件接口及控制器集成的工作空间。仓库包含机器人模型、控制器配置、真机启动文件和相关使用示例。

当前发布内容提供 **`gs_s1`（V3）** 的模型与配置。启动参数 `dexhand_type` 中的 `gs_s2` 是预留的型号选择接口；由于未提供对应模型和配置文件，当前不能使用该选项生成模型。

本仓库主要包含以下模块：

```text
ros2_botyard/
├── dexhand_bringup/                  # 控制器配置、真机启动文件和操作示例
│   ├── config/
│   ├── launch/
│   ├── rviz/
│   ├── src/
│   └── test/
├── dexhand_control/                  # ros2_control 硬件接口和自定义控制器
│   ├── hardwares/
│   │   └── dexhand_hardware_interface/
│   └── controllers/
│       ├── dexhand_controller/
│       └── dexhand_joint_controller/
├── dexhand_description/              # Xacro/URDF、网格、模型参数和模型资产
│   ├── config/
│   ├── launch/
│   ├── meshes/
│   ├── mjcf/
│   ├── rviz/
│   ├── urdf/
│   └── usd/
├── dexhand_interfaces/               # 自定义 ROS 2 msg/action 接口
└── debs/                             # 预编译 dexhand_driver 安装包
```

## 环境依赖

支持的 ROS 2 环境与仓库内驱动安装包如下：

| ROS 2 | Ubuntu | 驱动安装包 |
| --- | --- | --- |
| Jazzy | 24.04（amd64） | `debs/dexhand-driver.deb` |
| Humble | 22.04（amd64） | `debs/dexhand-driver_humble.deb` |

ROS 2 的安装请参考官方文档：[Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 或 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

安装构建工具及控制框架相关依赖：

```bash
sudo apt install python3-colcon-common-extensions

sudo apt-get install \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-generate-parameter-library \
  ros-${ROS_DISTRO}-pal-statistics
```

其中，[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) 用于辅助生成参数读取及校验相关代码。运行上述命令前，请确保已加载对应版本的 ROS 2 环境。

## 安装与编译

### 1. 创建工作空间并获取源码

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <仓库地址> ros2_botyard
```

后续示例均以仓库位于 `~/ros2_ws/src/ros2_botyard` 为前提。

### 2. 安装灵巧手驱动

从下面三条命令中，**选择与系统匹配的一条**执行：

```bash
# ROS 2 Jazzy / Ubuntu 24.04 / amd64
sudo apt install ~/ros2_ws/src/ros2_botyard/debs/dexhand-driver.deb

# ROS 2 Humble / Ubuntu 22.04 / amd64
sudo apt install ~/ros2_ws/src/ros2_botyard/debs/dexhand-driver_humble.deb
```

### 3. 编译并加载工作空间

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

如果需要在新终端自动加载环境，可在 `~/.bashrc` 中加入相应配置。例如，Jazzy 环境为：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

使用 Humble 时，将第一行的 `jazzy` 改为 `humble`。修改后重新打开终端，或执行 `source ~/.bashrc`。

如需配置 rosdep 的清华镜像，可按需设置：

```bash
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
```

### 4. 配置串口设备规则（真机使用）

仓库提供 `dexhand_description/config/99-botyard.rules`，用于配置灵巧手设备节点及访问规则：

```bash
sudo cp ~/ros2_ws/src/ros2_botyard/dexhand_description/config/99-botyard.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5. EtherCAT通信需配置免密(sudoers)

在Linux系统中使用EtherCAT通信接口要用到两个Capability：`CAP_NET_RAW` 和 `CAP_NET_ADMIN`。
默认的`cap_grant:=sudo`方案在launch文件中用`sudo -E /usr/bin/setpriv ...`临时借出，所以要求当前用户能**免密**调用`setpriv`。

```bash
sudo cp ~/ros2_ws/src/ros2_botyard/dexhand_description/config/99_ethercat_cap /etc/sudoers.d/
sudo chown root:root /etc/sudoers.d/99_ethercat_cap
sudo chmod 0440 /etc/sudoers.d/99_ethercat_cap

# 校验, 应看到: (root) NOPASSWD: SETENV: /usr/bin/setpriv
sudo -n -l | grep setpriv
```


## 快速开始

完成安装与编译后，在已加载工作空间环境的终端中使用以下命令。

### RViz 模型预览

如尚未安装关节状态发布器，可先安装对应 ROS 2 软件包：

```bash
sudo apt-get install ros-${ROS_DISTRO}-joint-state-*
```

启动带关节滑动条的模型预览：

```bash
ros2 launch dexhand_description play.launch.py
```

模型参数、右手显示、双手模型及 URDF 生成方法见 [dexhand_description/README.md](dexhand_description/README.md)。

### 真机启动

在驱动安装、设备连接及串口规则配置完成后，启动默认控制器：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py
```

默认以 `gs_s1` 模型启动，使用串口（RS485）连接，默认设备路径为 `/dev/dexhand`。控制链路涉及 `dexhand_controller`、`dexhand_hardware_interface` 和 `dexhand_driver`。

指定设备端口、通过EtherCAT连接、发送关节目标、执行手势或读取触觉反馈等操作，请参阅 [dexhand_bringup/README.md](dexhand_bringup/README.md)。发送动作指令前应确认设备已正确连接、关节运动空间无遮挡。

## 其他模型资产

本仓库还包含 `gs_s1` 对应的模型资产：

| 格式 / 平台 | 仓库位置 |
| --- | --- |
| MuJoCo（MJCF） | `dexhand_description/mjcf/gs_s1/` |
| Isaac Sim（USD） | `dexhand_description/usd/isaac_dexhand/` |

## 相关文档

| 文档 | 内容 |
| --- | --- |
| [dexhand_bringup/README.md](dexhand_bringup/README.md) | 真机启动、控制模式、手势 Action、触觉反馈与双手控制 |
| [dexhand_control/README.md](dexhand_control/README.md) | 硬件接口与自定义控制器的模块说明 |
| [dexhand_description/README.md](dexhand_description/README.md) | 模型结构、Xacro 参数、URDF 生成与预览 |
