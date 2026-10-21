# dexhand_bringup

## 概述

`dexhand_bringup` 提供灵巧手的综合 launch 文件、控制器配置和操作示例，用于将 `ros2_control` 与真机控制、RViz 预览连接起来。

本页依次介绍型号选择、真机启动、控制器指令、手势与触觉、双手控制。

## 型号选择

启动参数 `dexhand_type` 用于选择模型：

- `gs_s1`：V3，当前默认型号；不指定 `dexhand_type` 时使用该型号。
- `gs_s2`：预留选择接口；当前没有对应的模型与配置资源，无法使用该选项生成模型。

## 控制器概览

| 控制器 | 功能 | 主要启动入口 |
| --- | --- | --- |
| `dexhand_controller` | 处理关节位置、速度、力矩及指尖力相关控制指令；默认控制器 | `hw_dexhand_controller.launch.py` |
| `position_controller` | 使用 ros2_control 的位置控制器，接收关节位置目标 | `hw_position_controller.launch.py` |

以下示例的目标关节顺序以相应的 `config/*.yaml` 为准。向真机发送非零目标前，请确认实际关节限位、设备状态和运动空间。

## 真机启动

### 默认串口连接

默认使用 `gs_s1` 模型，通过串口（RS485）连接设备：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py
```

### 指定串口设备

例如使用 `/dev/ttyACM0`：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_mock_dexhand:=false port_name:=/dev/ttyACM0'
```

### EtherCAT 连接

不指定网卡时，驱动按现有配置自动探测：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_mock_dexhand:=false communicate:=EtherCAT'
```

需要指定网卡时，可先用 `ip -br link` 查询网卡名称，再设置 `port_name`：


```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_mock_dexhand:=false communicate:=EtherCAT port_name:=enp3s0'
```

`communicate:=EtherCAT` 选择 EtherCAT 后端，此时 `port_name` 表示**网卡名称**，不是 `/dev/tty*` 串口路径。


## dexhand_controller（默认控制器）

`dexhand_controller` 实现对指尖力(tactiles)、关节的位置/速度/力矩的统一控制。启动该控制器后，通过 `/dexhand_controller/commands` 发送 `std_msgs/msg/Float64MultiArray` 指令。

### 控制模式

| 模式 | 指令内容 |
| --- | --- |
| `position_only` | 仅包含 18 个主动关节的位置目标，其他数据使用默认值 |
| `position+tactile` | 包含 18 个关节位置目标与 5 个指尖力目标 |
| `full_command` | 包含关节位置、速度、力矩及 5 个指尖力目标 |

关节顺序与 `config/dexhand_controller.yaml` 的 `joints` 配置一致；指尖力顺序与其中 `tactiles` 的配置一致，依次为 `THT0`、`FFT0`、`MFT0`、`RFT0`、`LFT0`。原示例中的指尖力以 **0.01 N** 为单位。

对于 `full_command`，以下示例使用的关节三元组格式为「位置（rad）、速度（rad/s）、effort」。其中 effort 的实际使用建议遵循对应驱动及控制器配置；示例在空载条件下采用 `2000.0` 作为常用参考值。

``` bash
# 选项1：真机测试
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py

# 选项2：指定端口号为/dev/ttyACM0
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false port_name:=/dev/ttyACM0'

# 选项3：EtherCAT真机测试，不传网卡名，自动探测
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT'

# 选项4：EtherCAT真机测试，指定网卡(网卡名可用 ip -br link 查询)
# 说明：communicate:=EtherCAT 走 EtherCAT 后端，port_name 表示网卡名(不是 /dev/xxx 串口路径)。
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT port_name:=enp3s0'

```

### position_only：关节位置

示例位置指令：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.2618, 0.0,                    # FAJ3, FAJ1 (rad ,  rad)
    0.2618, 0.0618, 0.5236, 0.5236, # THJ4-1
    0.0, 0.5236, 1.0472,            # FFJ4-2
    0.0, 0.5236, 1.0472,            # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # LFJ4-2 (0°, 30°, 60°)
  ]
}"
```

复位指令：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 0.0,           # FAJ3, FAJ1 (rad ,  rad)
    0.0, 0.0, 0.0, 0.0, # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,      # FFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # MFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # RFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # LFJ4-2 (0°, 0°, 0°)
  ]
}"
```

### position+tactile：关节位置与指尖力

示例位置及指尖力指令（五个指尖力目标在数据末尾）：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.2618, 0.0,                    # FAJ3, FAJ1 (rad ,  rad)
    0.2618, 0.0618, 0.5236, 0.5236, # THJ4-1
    0.0, 0.5236, 1.0472,            # FFJ4-2
    0.0, 0.5236, 1.0472,            # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # LFJ4-2 (0°, 30°, 60°)
    100.0, 100.0, 100.0, 100.0, 100.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

复位指令：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 0.0,           # FAJ3, FAJ1 (rad ,  rad)
    0.0, 0.0, 0.0, 0.0, # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,      # FFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # MFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # RFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # LFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0, 0.0, 0.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

### full_command：完整控制指令

下列命令采用关节「位置、速度、effort」交错排列，并在末尾添加五个指尖力目标。
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.2618, 3.14, 2000.0,  0.0, 0.0, 0.0,                                                     # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.2618, 1.0, 2000.0,   0.0618, 1.0, 2000.0,   0.5236, 1.0, 2000.0,  0.5236, 1.0, 2000.0,  # THJ4-1
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # FFJ4-2
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # LFJ4-2 (0°, 30°, 60°)
    100.0, 100.0, 100.0, 100.0, 100.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

复位指令：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 3.14, 0.0,  0.0, 0.0, 0.0,                                   # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,  0.0, 1.0, 0.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # LFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0, 0.0, 0.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

握拳示例(注意配置好关节速度，避免手指碰撞造成关节堵转，导致灵巧手损坏)：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 0.0, 0.0,         0.0, 0.14, 0.0,                                                     # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    -0.4, 0.5, 1000.0,     0.8236, 3.14, 1000.0,   1.1854, 0.5, 2000.0,  0.5236, 0.5, 2000.0,  # THJ4-1
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 1000.0,                       # FFJ4-2
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # MFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # RFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # LFJ4-2 (0°, 90°, 90°)
    100.0, 100.0, 100.0, 100.0, 100.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

握拳后复位示例：
```bash
ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 0.0, 0.0,     0.0, 3.14, 2000.0,                                          # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.0, 1.0, 2000.0,  0.0, 1.0, 2000.0,   0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # LFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0, 0.0, 0.0,   # THT0, FFT0, MFT0, RFT0, LFT0 ( Unit: 0.01N )
  ]
}"
```

## 手势控制与触觉反馈

### 手势 Action

手势接口为 `dexhand_interfaces/action/DoGesture`，需要连接真机运行, 不支持在 Mock 模式下执行。
手势预设文件为 `/usr/local/share/dexhand_driver/gesture_presets.conf`。
当有动作在执行，新 goal 会被拒绝。

常用手势 ID：

| ID | 动作 | ID | 动作 |
| --- | --- | --- | --- |
| `1` | 五指握 | `2` | 四指握 |
| `3` | 三指握 | `4` | 四指扣握 |
| `5` | 五指扣握 | `6` | 二指捏合 |
| `7` | 三指捏合 | `8` | 爪式 |
| `9` | 夹式 | `10` | 钳式 |
| `255` | 张开 | | |

在真机控制栈已经启动后，执行以下示例：

```bash
# 五指握
ros2 action send_goal /dexhand_controller/do/gesture dexhand_interfaces/action/DoGesture "{gesture_id: 1}" --feedback

# 张开
ros2 action send_goal /dexhand_controller/do/gesture dexhand_interfaces/action/DoGesture "{gesture_id: 255}" --feedback
```

- 取消：命令行在 `send_goal` 终端按 Ctrl+C（Jazzy 支持），C++ 客户端用 `async_cancel_goal(goal_handle)`；收尾完成后返回 `CANCELED` 和业务结果 `CANCELLED(6)`。这仅代表执行线程已退出，通信故障时不保证机械手已物理停止。
- 硬件超时返回 `ABORTED` 和业务结果 `TIMEOUT(3)`。

#### 指尖触觉

##### 指尖合力
指尖触觉`合力数据`发布至 `/dexhand_controller/state/tactiles`，消息类型为 `std_msgs/msg/Float64MultiArray`。反馈包括 x、y 方向的切向力及 z 方向的法向力，顺序参见 `config/dexhand_controller.yaml` 的 `tactiles` 配置，单位为 **0.01 N**。

```bash
ros2 topic echo /dexhand_controller/state/tactiles
```

##### 触觉阵列
当你使用EtherCAT的通信接口来启动灵巧手，那么还会以100Hz频率发布指尖触觉`阵列数据`至 `/botyard_dexhand_hardware_tactile/grid_tactiles`，消息类型为 `dexhand_interfaces/msg/TactileGrid`。
数据格式：5根手指 × 116点阵，每个点包含fx/fy/fz三个方向的力，数据类型为`int8`。

```bash
ros2 topic echo /botyard_dexhand_hardware_tactile/grid_tactiles
```

