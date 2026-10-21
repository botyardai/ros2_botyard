# Introduction

Implements the hardware_interface related interfaces of the ros2_control framework.

## dexhand_hardware_interface

This is the dexterous hand hardware logic based on the ros2_control framework. Typically, it is introduced in the <ros2_control> tag of the URDF file.

### Dependency dexhand_driver

dexhand_driver is the underlying driver for the dexterous hand, implementing RS485, CANFD, EtherCAT communication protocols. Please follow the repository README file instructions to install it in advance.


## gz_ros2_control

Directly control the joints of the dexterous hand using the [position controller](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-userdoc) provided by ros2_control.

## controller_manager Real-Time Issue
Please ensure you follow the [Controller Manager real-time configuration](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html).
