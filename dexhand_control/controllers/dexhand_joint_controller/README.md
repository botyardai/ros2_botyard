# Introduction 

实现dexhand_joint_controller控制器，转发命令到一组关节的位置/速度/力矩接口。

## dexhand_joint_controller

基于forward_controllers_base实现一个简单的控制器，类似forward_command_controller, 但允许向topic同时发送关节的位置/速度/力矩指令，进而控制灵巧手运动。

```bash
# 手动启动控制器
$ ros2 run controller_manager spawner dexhand_joint_controller -p $(ros2 pkg prefix dexhand_bringup)/share/dexhand_bringup/config/dexhand_joint_controller.yaml

# 通过如下指令给dexhand_joint_controller发送控制指令(只允许指定主动关节的位置, 共18个关节数据)
# 目标值顺序与dexhand_bringup/config/dexhand_joint_controller.yaml的joints和interface_names配置一致

# 1. 只发送位置(需要修改dexhand_bringup/config/dexhand_joint_controller.yaml中的interface_names)
# (注意：需要确认interface_names中 *没有启用*: velocity, effort)
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 1}, {label: 'interface', size: 1, stride: 1}]},
  data: [
    0.2618, 0.0,                    # FAJ3, FAJ1 (rad ,  rad)
    0.2618, 0.2618, 0.5236, 0.5236, # THJ4-1 (15°, 15°, 30°, 30°)
    0.0, 0.5236, 1.0472,            # FFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.5236, 1.0472,            # LFJ4-2 (0°, 30°, 60°)
  ]
}"
# 复位
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 1}, {label: 'interface', size: 1, stride: 1}]},
  data: [
    0.0, 0.0,           # FAJ3, FAJ1 (rad ,  rad)
    0.0, 0.0, 0.0, 0.0, # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 0.0, 0.0,      # FFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # MFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # RFJ4-2 (0°, 0°, 0°)
    0.0, 0.0, 0.0,      # LFJ4-2 (0°, 0°, 0°)
  ]
}"

# 2. 发送位置和速度(需要修改dexhand_bringup/config/dexhand_joint_controller.yaml中的interface_names)
# (注意：需要确认interface_names中 *没有启用*: effort)
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 2}, {label: 'interface', size: 2, stride: 1}]},
  data: [
    0.2618, 3.14,  0.0, 0.0,                                   # FAJ3, FAJ1 (rad | rad/s,  rad | rad/s)
    0.2618, 3.14,  0.2618, 3.14,  0.5236, 1.0,   0.5236, 1.0,  # THJ4-1 (15°, 15°, 30°, 30°)
    0.0, 0.0,      0.5236, 3.14,  1.0472, 3.14,                # FFJ4-2 (0°, 30°, 60°)
    0.0, 0.0,      0.5236, 3.14,  1.0472, 3.14,                # MFJ4-2 (0°, 30°, 60°)
    0.0, 0.0,      0.5236, 3.14,  1.0472, 3.14,                # RFJ4-2 (0°, 30°, 60°)
    0.0, 0.0,      0.5236, 3.14,  1.0472, 3.14,                # LFJ4-2 (0°, 30°, 60°)
  ]
}"
# 复位
$ ros2 topic pub --once /dexhand_joint_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joints', size: 18, stride: 2}, {label: 'interface', size: 2, stride: 1}]},
  data: [
    0.0, 3.14,  0.0, 3.14,                        # FAJ3, FAJ1 (rad | rad/s,  rad | rad/s)
    0.0, 1.0,   0.0, 1.0,   0.0, 1.0,  0.0, 1.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14,  0.0, 3.14,  0.0, 3.14,            # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14,  0.0, 3.14,  0.0, 3.14,            # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14,  0.0, 3.14,  0.0, 3.14,            # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14,  0.0, 3.14,  0.0, 3.14,            # LFJ4-2 (0°, 0°, 0°)
  ]
}"

# 3. 发送位置、速度和手指关节力矩(需要修改dexhand_bringup/config/dexhand_joint_controller.yaml中的interface_names)
# (注意：需要确认interface_names中*同时启用*了: position, velocity, effort)
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
    0.0, 3.14, 0.0,  0.0, 0.0, 0.0,                                   # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,  0.0, 1.0, 0.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,  0.0, 3.14, 0.0,  0.0, 3.14, 0.0,                 # LFJ4-2 (0°, 0°, 0°)
  ]
}"

# 4. 握拳手势 (注意配置好关节速度，避免手指碰撞造成关节堵转，导致灵巧手损坏)
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
    0.0, 0.0, 0.0,     0.0, 3.14, 2000.0,                                          # FAJ3, FAJ1 (rad | rad/s | 0.01N,  rad | rad/s | 0.01N)
    0.0, 1.0, 2000.0,  0.0, 1.0, 2000.0,   0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,  # THJ4-1 (0°, 0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # FFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # MFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # RFJ4-2 (0°, 0°, 0°)
    0.0, 3.14, 0.0,    0.0, 3.14, 2000.0,  0.0, 3.14, 2000.0,                      # LFJ4-2 (0°, 0°, 0°)
  ]
}"

```
