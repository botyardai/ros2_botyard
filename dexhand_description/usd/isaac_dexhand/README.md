# 单手 Isaac Sim

本目录是一个独立的右手仿真环境，正常启动只需要本目录的 `right_hand.usd`，
不需要编译抓取插件。

右手竖直立在地面上，前臂底座通过固定关节连接世界坐标系。默认使用从
`delivery_sim.usd` 复制的 ROS 控制图：接收 JointState 命令，经 Articulation
Controller 下发目标，PhysX/PD 驱动关节运动。位置命令的角度单位为弧度。

## 启动与控制

推荐环境为 **Ubuntu 24.04 + Isaac Sim 5.1 官方安装包 + ROS 2 Jazzy**。
无需新建 Conda 环境，也不需要快递环境的 `dexhand_interfaces` 自定义接口。

新开 Bash 终端，退出已激活的 Conda 环境后执行：

```bash
cd ~/isaac_sim/isaac_dexhand
bash ./start_sim.sh
```

脚本自动查找 Isaac Sim 5.1、配置 ROS 环境、启用 `isaacsim.ros2.bridge`，
并打开同目录的 `right_hand.usd`。加载完成后点击 **Play**。
无需手动配置环境变量；这些配置只影响本次启动。

如果找不到 Isaac Sim，指定包含 `isaac-sim.sh` 和 `python.sh` 的安装目录：

```bash
bash ./start_sim.sh --isaac-dir /实际路径/isaacsim
```

`bash ./start_sim.sh --check` 仅检查路径，不启动程序。
脚本和场景根据自身位置定位，可整体移动本目录，也可从其他工作目录调用脚本。
使用的是 Isaac 自带的 Jazzy Bridge，无需编译快递抓取接口；外部 ROS 控制终端需要安装 Jazzy。
Isaac 与 ROS 控制终端的 `ROS_DOMAIN_ID` 必须一致，脚本默认使用 `14`，可在启动前修改：

```bash
ROS_DOMAIN_ID=15 bash ./start_sim.sh
```

另开终端，发送食指目标：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=14
ros2 topic pub --once /isaac_sim/joint_command sensor_msgs/msg/JointState \
  "{name: ['openarm_right_dexhand_FFJ3', 'openarm_right_dexhand_FFJ2'], position: [0.5, 0.5]}"
```

查看真实仿真反馈：

```bash
ros2 topic echo /isaac_sim/joint_states
ros2 topic echo /clock
```

| 话题 | 类型 | 用途 |
|---|---|---|
| `/isaac_sim/joint_command` | `sensor_msgs/msg/JointState` | 关节名称和目标角度 |
| `/isaac_sim/joint_states` | `sensor_msgs/msg/JointState` | 22 个关节的实际状态 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间 |

可以只发送部分主动关节，但必须使用带 `openarm_right_dexhand_` 前缀的全名，
且 `name` 与 `position` 数组逐项对应。位置控制只填写 `position`，不同时发送
`velocity/effort`。该 ROS 图与源场景一样直接执行输入，调用方应保证角度有限且
不超出关节限位。完整机器人命令中包含的左手或机械臂关节不属于本场景。

USD 中保留 `/Graph/ROS_JointStates` 和 `/Graph/ROS_Clock`。关节图的 namespace
仍为 `isaac_sim`，targetPrim 改为单手的物理根连杆
`/World/Hand/openarm_right_dexhand_fabase`。

## 模型与 PD 参数

- `right_hand.usd`：独立场景，约 21 MiB，包含右手、地面、灯光和观察相机。
  网格已内嵌；只使用 Isaac 自带材质，不依赖原仓库、双臂或在线场景资产。
- `start_sim.sh`：查找 Isaac Sim、配置 ROS Bridge 并启动场景。
- `load_scene.py`：在 Isaac 中打开同目录的 `right_hand.usd`。
- `README.md`：启动、位置控制及关节说明。

主动关节顺序：

```text
FAJ3 FAJ1
THJ4 THJ3 THJ2 THJ1
FFJ4 FFJ3 FFJ2
MFJ4 MFJ3 MFJ2
RFJ4 RFJ3 RFJ2
LFJ4 LFJ3 LFJ2
```

`FFJ1/MFJ1/RFJ1/LFJ1` 保留源场景的 PhysX mimic 关系，各自跟随对应的 J2。
它们不接受独立位置目标。

18 个驱动的 stiffness、damping、maxForce、type 均原样复制；没有通过
`set_gains()` 二次覆盖。物理频率为源场景的 200 Hz，关节求解迭代和
自碰撞设置也从源场景保留。底座的刚性固定保证手立在地上，不模拟自由平衡。

正常运行直接使用已打包的 `right_hand.usd`。
同一 ROS 域内仅运行一个使用上述话题的仿真场景，避免控制命令同时发送到多个场景。
