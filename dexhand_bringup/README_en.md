# Introduction

Provides several integrated launch files, e.g. moveit -> ros2_control -> real robot/simulation

## Simulation Control

Simulation-related control based on Gazebo, with launch files prefixed by `gz_`.

## ros_gz_bridge Data Pass-through

Directly bridges ROS and Gazebo data channels. On the Gazebo side, the official plugin publishes joint states to /joint_states.

```bash
# Use Gazebo simulation (with damping/friction and other physical properties)
$ ros2 launch dexhand_bringup gz_bridge.launch.py

# Directly set joint positions in Gazebo
$ gz topic -t /joint_position_cmd -m gz.msgs.Double -p 'data: 1.2'

# Or bridge ROS commands to the Gazebo world via ros_gz_bridge
$ ros2 topic pub /joint_position_cmd std_msgs/msg/Float64 "data: 1.2" -s -1
```

## Integrate ros2_control

Introduce the ros2_control framework into the control flow, using the official gz_ros2_control plugin to control the dexterous hand in Gazebo simulation. use position_controller to forward target positions for each joint of the dexterous hand.

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

## Integrate MoveIt

Use MoveIt as the motion planner, specify target points, generate joint trajectories, and use ros2_control's joint_trajectory_controller for precise dexterous hand motion control.

```bash
$ ros2 launch dexhand_bringup gz_moveit.launch.py

# Send control commands to the custom controller
# In Rviz, use the Motion Planning plugin on the left to select target points.
```

## Interact with the dexterous hand via Desktop APP (target following)

On the desktop APP, select the "3D Model" menu to enter a 3D space. Here you can select a target point, and the backend controller will solve for the dexterous hand to grasp the point.

### Dependency Installation: rosbridge

[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) is a tool for communication between ROS and the Web. It allows message passing between ROS2 nodes and browsers, enabling frontend APP interaction with the robot.

```bash
$ sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite
```

### Start ROS Backend Service and Interact with Desktop APP

```bash
# Based on Gazebo simulation
$ ros2 launch dexhand_bringup gz_target_following.launch.py

# Based on real robot
$ ros2 launch dexhand_bringup hw_target_following.launch.py
```

These launch files start the rosbridge_server service, which listens on port 9090 by default, waiting for the desktop juexian-app to send target points.

Open the desktop Juexian-APP, connect via WebSocket, enter the 3D model interface, and click "Subscribe Joint State" to receive robot status information from the backend for animation display.

To manually start rosbridge_server, use:
```bash
$ ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

## position_controller

Directly control dexterous hand joints using the [position controller](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-userdoc) provided by ros2_control.

```bash
# Option 1: Simulation debugging
$ ros2 launch dexhand_bringup gz_position_controller.launch.py

# Option 2: Animate joint position commands in rviz
# In Rviz, switch RobotModel topic to: /parallel/dexhand/robot_description, set TF_Prefix to: parallel/dexhand
# Set Global Option's Fixed Frame to: parallel/dexhand/world
# (Ignore RViz errors for now)
$ ros2 launch dexhand_bringup parallel_cmd_to_joint_state.launch.py

# Option 3: Real robot test, teleoperation glove captures hand joint angles and sends to position_controller
$ ros2 launch dexhand_bringup hw_position_controller.launch.py
$ ros2 launch dexhand_bringup hw_position_controller.launch.py namespace:=my_dexhand

# After choosing any option above, send control commands to position_controller (only specify active joint positions, 18 joints in total)
# The order of target values matches the joints configuration in the controller config file
$ ros2 topic pub -r 1 /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joint_list', size: 18, stride: 18}, {label: 'joint', size: 1, stride: 1}]},
  data: [
    0.0, 0.0,                 # FAJ3, FAJ1
    0.0, 0.0, 1.0472, 1.0472, # THJ4-1 (0°, 0°, 60°, 60°)
    0.2618, 0.7854, 0.7854,   # FFJ4-2 (15°, 45°,45°)
    0.0, 1.5708, 1.5708,      # MFJ4-2 (0°, 90°, 90°)
    0.0, 0.0, 0.0,            # RFJ4-2 (0°)
    0.2618, 0.4363, 0.4363,   # LFJ4-2 (15°, 25°,25°)
  ]
}"

$ ros2 topic pub -r 1 /dexhand_position_controller/commands std_msgs/msg/Float64MultiArray "{
  layout: {dim: [{label: 'joint_list', size: 18, stride: 18}, {label: 'joint', size: 1, stride: 1}]},
  data: [
    0.0, 0.0,            # FAJ3, FAJ1
    0.0, 0.0, 0.0, 0.0,  # THJ4-1 (0°,0°,0°,0°)
    0.0, 0.0, 0.0,  # FFJ4-2 (0°)
    0.0, 0.0, 0.0,  # MFJ4-2 (0°)
    0.0, 0.0, 0.0,  # RFJ4-2 (0°)
    0.0, 0.0, 0.0,  # LFJ4-2 (0°)
  ]
}"
```

### Teleoperation with Glove and Dexterous Hand

Real robot test: the teleoperation glove captures the finger tip pose and sends it to the /dexhand/multi_target_following topic, where the backend bio_ik solves for joint angles.

```bash
# Based on real robot
$ ros2 launch dexhand_bringup hw_tele_operation.launch.py
```

### Integrate two dexterous hands in one URDF file, but control them independently

```bash
ros2 launch dexhand_bringup hw_position_controller_two_dexhand.launch.py namespace:=two_dexhand lh_port_name:=/dev/ttyACM1 rh_port_name:=/dev/ttyACM0
```
Here, the `lh_port_name` and `rh_port_name` parameters specify the serial port numbers for the left and right hands, respectively.
