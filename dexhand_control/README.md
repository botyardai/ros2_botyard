# dexhand_control

## 概述

`dexhand_control` 包含与 `ros2_control` 相关的硬件接口和灵巧手自定义控制器实现。硬件接口负责将控制框架与设备驱动连接，控制器负责接收相应的关节控制指令。

## 目录结构

```text
dexhand_control/
├── hardwares/
│   └── dexhand_hardware_interface/   # 灵巧手 ros2_control 硬件接口
└── controllers/
    ├── dexhand_controller/          # 灵巧手自定义控制器
    └── dexhand_joint_controller/    # 关节控制器
```

## 主要模块

| 模块 | 说明 |
| --- | --- |
| `dexhand_hardware_interface` | 对接 `ros2_control` 和灵巧手驱动的硬件接口 |
| `dexhand_controller` | 统一处理关节位置、速度、力矩及指尖力相关控制指令 |
| `dexhand_joint_controller` | 面向关节位置、速度及力矩的控制器 |

控制器的启动文件和指令示例集中在 `dexhand_bringup` 中，本目录不重复维护控制命令。

## 使用说明

- [启动与控制示例](../dexhand_bringup/README.md)
- [dexhand_joint_controller 说明](controllers/dexhand_joint_controller/README.md)
- [项目安装与快速开始](../README.md)
