# dexhand_bringup

## 概述

`dexhand_bringup` 提供灵巧手的综合 launch 文件、控制器配置和操作示例，用于将 `ros2_control` 与真机控制、RViz 预览连接起来。

本页依次介绍型号选择、真机启动、控制器指令、手势与触觉、双手控制。完整的 EtherCAT 配置及验证说明请参阅 [README_Ethercat.md](README_Ethercat.md)。

## 型号选择

启动参数 `dexhand_type` 用于选择模型：

- `gs_s1`：V3，当前默认型号；不指定 `dexhand_type` 时使用该型号。
- `gs_s2`：预留选择接口；当前没有对应的模型与配置资源，无法使用该选项生成模型。

## 控制器概览

| 控制器 | 功能 | 主要启动入口 |
| --- | --- | --- |
| `dexhand_controller` | 处理关节位置、速度、力矩及指尖力相关控制指令；默认控制器 | `hw_dexhand_controller.launch.py` |
| `dexhand_joint_controller` | 联合控制关节位置、速度和力矩 | `hw_dexhand_joint_controller.launch.py` |
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

`communicate:=EtherCAT` 选择 EtherCAT 后端，此时 `port_name` 表示**网卡名称**，不是 `/dev/tty*` 串口路径。网卡选择、权限及连接配置详见 [README_Ethercat.md](README_Ethercat.md)。

## dexhand_controller（默认控制器）

`dexhand_controller` 用于处理关节及指尖力相关的控制命令。启动该控制器后，通过 `/dexhand_controller/commands` 发送 `std_msgs/msg/Float64MultiArray` 指令。

### 控制模式

| 模式 | 指令内容 |
| --- | --- |
| `position_only` | 仅包含 18 个主动关节的位置目标，其他数据使用默认值 |
| `position+tactile` | 包含 18 个关节位置目标与 5 个指尖力目标 |
| `full_command` | 包含关节位置、速度、力矩及 5 个指尖力目标 |

关节顺序与 `config/dexhand_controller.yaml` 的 `joints` 配置一致；指尖力顺序与其中 `tactiles` 的配置一致，依次为 `THT0`、`FFT0`、`MFT0`、`RFT0`、`LFT0`。原示例中的指尖力以 **0.01 N** 为单位。

对于 `full_command`，以下示例使用的关节三元组格式为「位置（rad）、速度（rad/s）、effort」。其中 effort 的实际使用建议遵循对应驱动及控制器配置；示例在空载条件下采用 `2000.0` 作为常用参考值。

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

握拳示例（执行前检查关节限位、速度及负载条件）：
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

手势接口为 `dexhand_interfaces/action/DoGesture`，需要连接真机运行；不能在 Mock 模式下执行。手势预设文件为 `/usr/local/share/dexhand_driver/gesture_presets.conf`。

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

Action 接单后反馈 `BUSY(4)`；最终返回成功、失败、超时或取消等执行结果。已有动作在执行或收尾期间，新 goal 会被拒绝。

- 在 Jazzy 中，可在 `send_goal` 的终端按 `Ctrl+C` 请求取消；C++ 客户端可调用 `async_cancel_goal(goal_handle)`。
- 取消完成后，Action 返回 `CANCELED`，业务结果为 `CANCELLED(6)`；这表示执行线程已退出，通信异常时不应据此认定机械手已经物理停止。
- 硬件超时对应 `ABORTED` 和业务结果 `TIMEOUT(3)`。

有关完整链路的验证说明，参阅 [README_Ethercat.md](README_Ethercat.md) 中的「手势调用与取消」。

### 指尖触觉反馈

`dexhand_controller` 将指尖触觉反馈发布至 `/dexhand_controller/state/tactiles`，消息类型为 `std_msgs/msg/Float64MultiArray`。反馈包括 x、y 方向的切向力及 z 方向的法向力，顺序参见 `config/dexhand_controller.yaml` 的 `tactiles` 配置，单位为 **0.01 N**。

```bash
ros2 topic echo /dexhand_controller/state/tactiles
```

## 其他控制器

### ~~dexhand_joint_controller~~

此控制器用于联合下发关节位置、速度与力矩目标，以下保留完整的指令示例。

启动控制器：

```bash
ros2 launch dexhand_bringup hw_dexhand_joint_controller.launch.py
```

在 `config/dexhand_joint_controller.yaml` 中，需根据实际控制任务确认 `interface_names` 已启用 `position`、`velocity`、`effort`。以下消息按 18 个关节、每关节 3 个接口组织，使用 `Float64MultiArray` 的 `layout` 说明维度。示例在空载条件下以 `2000.0` 作为 effort 参考值。

位置、速度与力矩示例：
```bash
ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 3}, {label: 'interface', size: 3, stride: 1}]},
  data: [
    0.2618, 3.14, 2000.0,  0.0, 0.0, 0.0,                                                     # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.2618, 1.0, 2000.0,   0.2618, 1.0, 2000.0,   0.5236, 1.0, 2000.0,  0.5236, 1.0, 2000.0,  # THJ4-1 (15°, 15°, 30°, 30°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # FFJ4-2 (0°, 30°, 60°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.0, 0.0,         0.5236, 3.14, 2000.0,  1.0472, 3.14, 2000.0,                       # LFJ4-2 (0°, 30°, 60°)
  ]
}"
```

复位指令：
```bash
ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 3}, {label: 'interface', size: 3, stride: 1}]},
  data: [
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,                                  # FAJ3, FAJ1 (rad | rad/s | 0.01N, )
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,                  # FFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,                  # MFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,                  # RFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,                  # LFJ4-2 (0°, 0°, 0°)
  ]
}"
```

握拳示例（需确认速度、力矩与关节限位）：
```bash
ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 3}, {label: 'interface', size: 3, stride: 1}]},
  data: [
    0.0, 0.0, 0.0,         0.2618, 3.14, 2000.0,                                               # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.2618, 3.14, 2000.0,  0.5236, 3.14, 2000.0,   0.7854, 0.5, 2000.0,  0.5236, 0.5, 2000.0,  # THJ4-1 (15°, 30°, 45°, 30°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # FFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # MFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # RFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,         1.5708, 3.14, 2000.0,   1.5708, 3.14, 2000.0,                       # LFJ4-2 (0°, 90°, 90°)
  ]
}"
```

握拳后复位示例：
```bash
ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 3}, {label: 'interface', size: 3, stride: 1}]},
  data: [
    0.0, 0.0, 0.0,   0.0, 3.14, 0.0,                                    # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                   # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                   # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                   # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                   # LFJ4-2 (0°, 0°, 0°)
  ]
}"
```

### position_controller

此控制器采用 [ros2_control 的位置控制器](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-userdoc)，接收关节位置命令。主要用于模型预览和简化控制测试，不推荐作为真机的默认控制方式。

通过 RViz 展示关节位置指令：

```bash
ros2 launch dexhand_bringup parallel_cmd_to_joint_state.launch.py
```

真机启动入口：

```bash
ros2 launch dexhand_bringup hw_position_controller.launch.py
```

需要命名空间、右手模型及指定串口时：

```bash
ros2 launch dexhand_bringup hw_position_controller.launch.py namespace:=my_dexhand xacro_args:='use_ros2_control:=true use_mock_dexhand:=false use_left_hand:=false port_name:=/dev/ttyACM0'
```

消息发布至 `/dexhand_position_controller/commands`，类型为 `std_msgs/msg/Float64MultiArray`；数据顺序按 `config/dexhand_position_controller.yaml` 的 `joints` 配置确定。

关节位置示例：
```bash
ros2 topic pub --once /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 1}, {label: 'interface', size: 1, stride: 1}]},
  data: [
    0.2618, 0.0,                    # FAJ3, FAJ1 (15°, 0°)
    0.2618, 0.2618, 0.7854, 0.7854, # THJ4-1 (15°, 15°, 45°, 45°)
    0.0, 0.5236, 1.0472,            # FFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # LFJ4-2 (0°, 30°, 60°)
  ]
}"
```

复位示例：
```bash
ros2 topic pub --once /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 1}, {label: 'interface', size: 1, stride: 1}]},
  data: [
    0.0, 0.0,            # FAJ3, FAJ1 (0°, 0°)
    0.0, 0.0, 0.0, 0.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,       # FFJ4-2 (0°)
    0.0, 0.0, 0.0,       # MFJ4-2 (0°)
    0.0, 0.0, 0.0,       # RFJ4-2 (0°)
    0.0, 0.0, 0.0,       # LFJ4-2 (0°)
  ]
}"
```

使用 `parallel_cmd_to_joint_state.launch.py` 时，默认命名空间为 `parallel/dexhand`。当命名空间非空时，`description.launch.py` 会相应调整 RViz 的 Fixed Frame、RobotModel Description Topic 和 TF Prefix；如需自定义显示，可在 RViz 中检查这些配置。

## 双手控制

在同一个 URDF 中组合两只灵巧手，同时保持各自的控制入口：

```bash
ros2 launch dexhand_bringup hw_position_controller_two_dexhand.launch.py namespace:=two_dexhand lh_port_name:=/dev/ttyACM1 rh_port_name:=/dev/ttyACM0
```

`lh_port_name` 和 `rh_port_name` 分别指定左手、右手的串口设备。

若两只手均使用 EtherCAT，可采用**两块网卡、两个终端、两个独立命名空间**的方式分别启动控制实例。具体网卡设置和使用约束见 [README_Ethercat.md](README_Ethercat.md) 中「从站识别依据」一节。

## EtherCAT 详细说明

使用 `communicate:=EtherCAT` 时，`ros2_control_node` 中的 `DexhandHardwareInterface` 负责设备连接，应用层通过 ROS 接口交互。关于网卡选择、链路参数、权限、启动和恢复方式，以及手势链路验证，请统一参阅 [README_Ethercat.md](README_Ethercat.md)。
