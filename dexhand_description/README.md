# dexhand_description

## 概述

`dexhand_description` 用于管理灵巧手的机器人描述文件与相关模型资源，包括 URDF、Xacro、Mesh、RViz 配置及模型参数。基础模型和关节配置维护在本包中；更具体的控制应用及启动流程由 `dexhand_bringup` 组织。

## 模型组织

### 模型组成

灵巧手模型拆分为 **hand** 与 **forearm** 两部分，并组合形成完整的 dexhand 模型。与 `ros2_control` 相关的关节参数统一存放在 `config/` 目录中。

### 模型版本

`dexhand_type` 用于选择模型版本：

- `gs_s1`：V3，当前默认型号。
- `gs_s2`：预留型号选择接口；当前未提供对应模型、网格和配置资源，不能通过该选项生成模型。

当前 `gs_s1` 资源主要位于：

| 目录 | 内容 |
| --- | --- |
| `urdf/gs_s1/` | Xacro 模型文件 |
| `meshes/gs_s1/` | 模型网格 |
| `config/gs_s1/` | 关节限位等参数 |

`config/joint_limits.yaml` 是指向 `gs_s1/joint_limits.yaml` 的兼容符号链接。未指定 `dexhand_type` 时，使用默认的 V3 模型。

## Xacro 文件说明

### 纯 `.xacro` 文件

模型通过 SolidWorks 的 `sw2urdf` 插件导出为 URDF。导出的原生模型内容保存在 `.xacro` 文件中，例如 `left_hand_macro.xacro` 和 `forearm_macro.xacro`。

### `.urdf.xacro` 文件

此类文件在基础模型之上按需组合功能配置，例如：

- 增加 `world` link，方便复用 RViz 显示配置；
- 配置模型在 RViz 中的颜色；
- 增加 `ros2_control` 相关描述。

## 模型生成与预览

### 生成 URDF

在模型目录下执行：

```bash
cd ~/ros2_ws/src/ros2_botyard/dexhand_description/urdf

# 生成默认模型
xacro ./dexhand.urdf.xacro > dexhand.urdf

# 生成右手模型
xacro use_left_hand:=false ./dexhand.urdf.xacro > dexhand_rh.urdf
```

### 使用 launch 文件加载模型

`description.launch.py` 用于加载机器人描述，可通过 `urdf_package`、`urdf_path` 指定模型文件，通过 `xacro_args` 传递模型参数。

```bash
# 显示右手模型
ros2 launch dexhand_description description.launch.py xacro_args:='use_left_hand:=false '

# 加载双手组合模型
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/two_dexhand_as_whole.urdf.xacro'

# 启用 ros2_control 并使用 mock 硬件
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/two_dexhand_as_whole.urdf.xacro' xacro_args:='use_ros2_control:=true use_mock_dexhand:=true '
```

其中常用的 Xacro 参数包括：

| 参数 | 作用 |
| --- | --- |
| `use_left_hand` | 选择左手或右手模型，默认左手 |
| `use_ros2_control` | 是否加入 `ros2_control` 描述；控制灵巧手时需要启用 |
| `use_mock_dexhand` | 是否使用 `mock_components/GenericSystem` 模拟硬件 |

连接真机时，应启用 `use_ros2_control` 并关闭 `use_mock_dexhand`。具体启动请使用 [dexhand_bringup/README.md](../dexhand_bringup/README.md) 中的真机入口。

### 使用 RViz 与关节滑动条

```bash
# 默认左手
ros2 launch dexhand_description play.launch.py

# 右手
ros2 launch dexhand_description play.launch.py xacro_args:='use_left_hand:=false '
```

如需在不同命名空间内显示模型，可分别启动：

```bash
ros2 launch dexhand_description description.launch.py namespace:='lh' use_rviz:=true
ros2 launch dexhand_description description.launch.py namespace:='rh' use_rviz:=false
```

此时需要在 RViz 中配置对应的 `robot_description` 话题及 TF Prefix，以免不同命名空间的模型互相冲突。

## 模型更新与组合

### 更新导出的模型

修改对应的纯 `.xacro` 文件，找到以下标记：

```xml
<!-- pure URDF goes below -->
```

使用更新后的 URDF 内容替换标记下方的原始模型描述。

### 组合新模型

以 `dexhand.urdf.xacro` 为基础，根据需要调整引用的 Xacro 文件并删减模块。双手组合示例可参考 `two_dexhand_as_whole.urdf.xacro`。

## 模型格式转换

### SDF 导出

在 `dexhand_description/urdf` 目录中，先生成 URDF，再转换为 SDF：

```bash
cd ~/ros2_ws/src/ros2_botyard/dexhand_description/urdf
xacro ./dexhand.urdf.xacro > dexhand.urdf
gz sdf -p dexhand.urdf > dexhand.sdf
```

### MuJoCo（MJCF）转换说明

模型从 URDF 转换为 MJCF 时，需注意以下关节类型的表示差异：

1. `fixed` 关节在 MJCF 中不生成独立的 `<joint>` 元素，固定关系由 `<body>` 层级隐式表示。
2. `floating` 关节在 MJCF 中使用 `type="free"` 的 `<joint>`，仅适用于根节点，包含平移和旋转共六个自由度。
3. 对仅需要旋转的关节，直接在 URDF 中设置 `type="ball"` 并不能按该转换流程直接完成转换。
4. 对模型中的 `FALJ1`、`FARJ1`，对应的转换步骤是先用 `revolute` 表示，导出 MJCF 后再手动调整为 `<joint type="ball">`。

上述内容是本项目现有模型的转换说明；修改模型时，应结合实际关节定义检查转换结果。
