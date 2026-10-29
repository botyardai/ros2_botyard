# DexHand Isaac Sim 仿真资产

本目录提供左右手的 Isaac Sim 仿真资产。每只手由一个主场景 USD 和一个几何 USD 组成。
主场景包含刚体、关节、碰撞、材质绑定和 ROS 2 控制图；几何 USD 包含 visual/collision 网格。

## 1. 交付文件

| 文件 | 用途 |
|---|---|
| `right_hand.usd` | 右手主场景，推荐使用 `start_sim.sh` 启动 |
| `right_hand_geometry.usd` | 右手几何库，必须和 `right_hand.usd` 放在同一目录 |
| `left_hand.usd` | 左手主场景，可在 Isaac Sim 中直接打开 |
| `left_hand_geometry.usd` | 左手几何库，必须和 `left_hand.usd` 放在同一目录 |
| `start_sim.sh` | 自动查找 Isaac Sim 并启动右手场景的脚本 |
| `load_scene.py` | 启动脚本使用的场景加载脚本 |

使用时请打开 `right_hand.usd` 或 `left_hand.usd`，不要直接打开 `*_geometry.usd`。
主场景会自动引用对应的几何库。不要单独移动、改名或删除主场景与几何库中的任何一个文件。

## 2. 环境要求

- Ubuntu 24.04
- Isaac Sim 5.1
- NVIDIA GPU 和可用的 NVIDIA 驱动
- ROS 2 Jazzy（只有需要通过 ROS 2 控制时才必须安装）

不需要编译额外的抓取插件，也不需要创建 Conda 环境。

## 3. 启动右手

在本目录执行：

```bash
cd /path/to/isaac_dexhand
bash ./start_sim.sh
```

Isaac Sim 启动并打开场景后，点击界面上的 **Play** 开始仿真。

启动脚本会自动查找 Isaac Sim 5.1。如果没有找到，可以手动指定 Isaac Sim 安装目录：

```bash
bash ./start_sim.sh --isaac-dir /path/to/isaacsim
```

只检查安装路径和交付文件是否完整，不启动 Isaac Sim：

```bash
bash ./start_sim.sh --check
```

脚本默认使用 `ROS_DOMAIN_ID=14`。如果使用其他域，需要在启动和 ROS 控制终端中使用相同的值：

```bash
ROS_DOMAIN_ID=15 bash ./start_sim.sh
```

## 4. 打开左手

左手目前不由 `start_sim.sh` 默认启动。验证左手时，在 Isaac Sim 中选择：

```text
File → Open → left_hand.usd
```

打开后点击 **Play**。如果通过 ROS 2 控制左手，使用左手关节名称中的 `left_` 前缀。

同一 ROS 域内建议一次只运行一只手，因为左右手默认使用相同的 ROS 话题名称。

## 5. ROS 2 控制

右手启动后，ROS 2 话题为：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/isaac_sim/joint_command` | `sensor_msgs/msg/JointState` | 发送关节目标位置 |
| `/isaac_sim/joint_states` | `sensor_msgs/msg/JointState` | 查看仿真关节状态 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Isaac Sim 仿真时钟 |

在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=14
ros2 topic pub --once /isaac_sim/joint_command sensor_msgs/msg/JointState \
  "{name: ['right_FFJ3', 'right_FFJ2'], position: [0.5, 0.5]}"
```

注意：

- 角度单位是弧度；
- `name` 和 `position` 的顺序必须一一对应；
- 发送位置命令时只填写 `position`，不要同时填写 `velocity` 或 `effort`；
- 右手关节使用 `right_` 前缀，左手关节使用 `left_` 前缀；
- `FFJ1`、`MFJ1`、`RFJ1`、`LFJ1` 是 mimic 关节，跟随对应的 J2，不接受独立位置命令。

主动控制关节名称如下。实际发送时，在前面加上 `right_` 或 `left_`：

```text
FAJ3 FAJ1
THJ4 THJ3 THJ2 THJ1
FFJ4 FFJ3 FFJ2
MFJ4 MFJ3 MFJ2
RFJ4 RFJ3 RFJ2
LFJ4 LFJ3 LFJ2
```

查看反馈：

```bash
ros2 topic echo /isaac_sim/joint_states
```

## 6. 视口选择和资产保护

几何网格位于独立的 instanceable 几何库中。这样可以保护内部 Mesh，客户仍然可以通过 Link、刚体和关节进行仿真控制。

在 Isaac Sim 视口中建议将选择模式切换为：

```text
Selection Mode → Model Kind
```

此时点击手指或手掌网格会优先选中上层 Link，而不是直接进入 Mesh。

如果需要调整手指运动，请使用 Physics Inspector 或 ROS 关节命令；不要直接拖动或编辑几何库中的 Mesh。

## 7. 常见问题

### Isaac Sim 找不到

使用安装目录参数：

```bash
bash ./start_sim.sh --isaac-dir /path/to/isaacsim
```

该目录必须包含 `isaac-sim.sh`、`python.sh` 以及 Isaac Sim 5.1 的 ROS 2 Jazzy Bridge。

### 打开场景时提示找不到几何文件

确认下面的文件仍然在同一目录，并且文件名没有改变：

```text
right_hand.usd              right_hand_geometry.usd
left_hand.usd               left_hand_geometry.usd
```

### ROS 2 话题没有数据或关节不动

确认以下条件：

1. Isaac Sim 已点击 **Play**；
2. 控制终端和 Isaac Sim 使用相同的 `ROS_DOMAIN_ID`；
3. 话题使用 `/isaac_sim/joint_command`；
4. 关节名称使用正确的 `right_` 或 `left_` 前缀；
5. 发送的是弧度制位置，并且没有超过关节限位。



