# Introduction 

提供一些综合的launch启动文件。比如moveit -> ros2_control -> 真机/仿真

## 控制仿真

基于Gazebo做仿真的相关控制, launch文件都以`gz_`前缀打头。

## ros_gz_bridge透传

直接经bridge打通ROS和Gazebo的数据通道。在gazebo侧借助官方插件将关节状态发布到/joint_states.

``` bash
# 使用Gazebo仿真(有阻尼/摩檫力等物理特性了)
$ ros2 launch dexhand_bringup gz_bridge.launch.py

# 直接在gazebo中设置关节位置
$ gz topic -t /joint_position_cmd -m gz.msgs.Double -p 'data: 1.2'

# 或者通过ros_gz_bridge将ros指令桥接到gazebo世界中
$ ros2 topic pub /joint_position_cmd std_msgs/msg/Float64 "data: 1.2" -s -1
```

## 接入ros2_control

在整个控制流程中引入ros2_control框架，借助其官方的gz_ros2_control插件，实现对Gazebo世界里的灵巧手仿真控制。
我们借助框架提供的position_controller，它接收topic的指令，将其转换成灵巧手各个关节的目标位置。

``` bash
$ ros2 launch dexhand_bringup gz_position_controller.launch.py

# 给位置controller发送控制指令
$ ros2 topic pub --once /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
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

## 接入MoveIt

使用moveit作为运动规划器，给定目标点位，生成关节轨迹（joint trajectory），再借助ros2_control的joint_trajectory_controller实现对灵巧手运动的精细控制。

``` bash
$ ros2 launch dexhand_bringup gz_moveit.launch.py

# 给自定义controller发送控制指令
# 在Rviz中左侧有Motion Planning插件，可以选择目标点位。
```

## 通过桌面APP与灵巧手互动(target following)

在桌面APP上，选择`3D模型`菜单，进入一个3D空间。在这里可以选择一个目标点，经过后台控制器的解算，让灵巧手抓取这个点。

### 安装依赖: rosbridge

[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)是一个用于在ROS和Web之间进行通信的工具。它允许你在ROS2节点和浏览器之间传递消息，从而实现前端APP与机器人的交互。

```bash

$ sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite

```

### 启动ROS后台服务与桌面APP交互

```bash
# 基于Gazebo仿真
$ ros2 launch dexhand_bringup gz_target_following.launch.py

# 基于真机
$ ros2 launch dexhand_bringup hw_target_following.launch.py
```

这里launch文件会启动rosbridge_server服务默认在后台监听`9090`端口，等待桌面juexian-app下发目标点位。

打开桌面Juexian-APP, `WebSocket`方式连接。进入3D模型界面，点击`订阅关节状态`，就可以接收后台传递回来的机器人状态信息，进行动画展示。

如果需要手动启动rosbridge_server, 可以使用如下命令:
```bash
$ ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```


## dexhand_controller(推荐/默认控制器)

实现对指尖力(tactiles)、关节的位置/速度/力矩的统一控制，驱动灵巧手运动和抓握。

``` bash
# 选项1：真机测试
$ ros2 launch dexhand_bringup hw_dexhand_controller.launch.py
# 选项2：指定端口号为/dev/ttyACM0
$ ros2 launch dexhand_bringup hw_dexhand_controller.launch.py xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false port_name:=/dev/ttyACM0'


# 通过如下指令给dexhand_controller发送控制指令(只允许指定主动关节的位置, 共18个关节数据)
# 目标值顺序与dexhand_bringup/config/dexhand_controller.yaml的joints和tactiles配置一致
# 支持3种控制模式：
# - position_only: 仅给定关节位置，其余数据使用默认值
# - position+tactile: 仅给定关节位置和指尖力，其余数据使用默认值
# - full_command: 完整指令，包括关节位置、速度、力矩和指尖力(精细控制)
# (注意：
#    1. effort的值如果小于2000.0, 可能会导致角度运动不到位。所以建议空载情况下，effort值设置为2000.0
#    2. 指尖力单位为0.01N
# )

## position_only模式示例指令
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.2618, 0.0,                    # FAJ3, FAJ1 (rad ,  rad)
    0.2618, 0.0618, 0.5236, 0.5236, # THJ4-1
    0.0, 0.5236, 1.0472,            # FFJ4-2
    0.0, 0.5236, 1.0472,            # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # LFJ4-2 (0°, 30°, 60°)
  ]
}"
# 复位
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [
    0.0, 0.0,           # FAJ3, FAJ1 (rad ,  rad)
    0.0, 0.0, 0.0, 0.0, # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,      # FFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # MFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # RFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # LFJ4-2 (0°, 0°, 0°)
  ]
}"

## position+tactile模式示例指令
# (注意：指尖力单位为0.01N)
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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
# 复位
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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

# full_command模式示例指令, 用作精细控制：发送全量指令，包含位置、速度、手指关节力矩 和指尖力
# (注意：指尖力单位为0.01N)
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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
# 复位
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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

# full_command模式: 握拳手势 (注意配置好关节速度，避免手指碰撞造成关节堵转，导致灵巧手损坏)
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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
# 复位 (注意配置好关节速度，避免手指碰撞)
$ ros2 topic pub --once /dexhand_controller/commands std_msgs/msg/Float64MultiArray "{
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

dexhand_controller会将指尖触觉(x,y轴切向力和z轴法向力)发送到/dexhand_controller/state/tactiles topic, 消息类型为std_msgs::msg::Float64MultiArray
目标值顺序与dexhand_bringup/config/dexhand_controller.yaml的tactiles配置一致, 单位为0.01N

```bash
$ ros2 topic echo /dexhand_controller/state/tactiles 
```


## dexhand_joint_controller

基于botyard提供的dexhand_joint_controller支持`同时`控制灵巧手关节的`位置、速度和手指关节力矩`三个维度。

``` bash
# 选项1：真机测试
$ ros2 launch dexhand_bringup hw_dexhand_joint_controller.launch.py

# 选择上述任一选项后，通过如下指令给dexhand_joint_controller发送控制指令(只允许指定主动关节的位置, 共18个关节数据)
# 同时控制关节位置、速度和力矩, 目标值顺序与dexhand_bringup/config/dexhand_joint_controller.yaml的joints和interface_names配置一致
# (注意：
#    1. 需要确认interface_names中*启用*了: position, velocity, effort
#    2. effort的值如果小于2000.0, 可能会导致角度运动不到位。所以建议空载情况下，effort值设置为2000.0
# )
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
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

# 复位
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
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

# 握拳手势 (注意配置好关节速度和力矩，避免手指碰撞造成关节堵转，导致灵巧手损坏)
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
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
# 复位 (注意配置好关节速度，避免手指碰撞)
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
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

## position_controller(不建议在真机上使用，但可以用作仿真简单的测试)

基于ros2_control提供的[位置控制器](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-userdoc)直接控制灵巧手关节。

``` bash
# 选项1：仿真调试
$ ros2 launch dexhand_bringup gz_position_controller.launch.py

# 选项2：将下发的关节位置指令直接在rviz做动画展示
# 注意在Rviz的RobotModel中切换topic为: /parallel/dexhand/robot_description，同时将TF_Prefix设置为: parallel/dexhand
# 最后再将Global Option中的Fixed Frame设置为: parallel/dexhand/world
# （此时RViz上的报错可暂时忽略）
$ ros2 launch dexhand_bringup parallel_cmd_to_joint_state.launch.py

# 选项3：真机测试, 由遥操手套捕获手的关节角度变化，下发到position_controller
$ ros2 launch dexhand_bringup hw_position_controller.launch.py
# 启用namespace，并使用右手，指定灵巧手的串口号
$ ros2 launch dexhand_bringup hw_position_controller.launch.py namespace:=my_dexhand xacro_args:='use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false use_left_hand:=false port_name:=/dev/ttyACM0'

# 选择上述任一选项后，通过如下指令给position_controller发送控制指令(只允许指定主动关节的位置, 共18个关节数据)
# 这里的目标值顺序与该controller配置文件的joints关节配置项保持一致
# 只控制关节位置, 目标值顺序与dexhand_bringup/config/dexhand_position_controller.yaml的joints配置一致
$ ros2 topic pub --once /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
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

# 复位
$ ros2 topic pub --once /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
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


### 通过遥操手套与灵巧手互动(tele operation)

真机测试, 由遥操手套捕获手的finger_tip pose，下发到/dexhand/multi_target_following topic, 后台bio_ik解算关节角度。

```bash
# 基于真机
$ ros2 launch dexhand_bringup hw_tele_operation.launch.py
```

### 在同一个URDF文件里集成2只灵巧手, 但分别独立的控制它们

```bash

ros2 launch dexhand_bringup hw_position_controller_two_dexhand.launch.py namespace:=two_dexhand lh_port_name:=/dev/ttyACM1 rh_port_name:=/dev/ttyACM0

```
这里的`lh_port_name`和`rh_port_name`参数分别指定了左右手的串口号。
