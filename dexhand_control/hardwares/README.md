# Introduction 

实现ros2_control框架的hardware_interface相关接口。

## dexhand_hardware_interface

这是基于ros2_control框架下的灵巧手硬件逻辑。通常它是在URDF文件的<ros2_control>标签中被引入。

### 依赖 dexhand_driver

dexhand_driver是灵巧手的底层驱动，实现了RS485，CANFD, EtherCAT通信协议, 请遵循仓库README文件指引，提前安装好。 

## gz_ros2_control

基于ros2_control提供的[位置控制器](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-userdoc)直接控制灵巧手关节。

## controller_manager实时性问题
请确保按照[Controller Manager实时性配置](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html)
