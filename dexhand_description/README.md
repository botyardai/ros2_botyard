# dexhand_description

## 概述

本仓库主要用于管理灵巧手的建模，如URDF，Xacro，Mesh, .rviz文件等。

## 设计思路

灵巧手我们拆分成 hand 和 forearm 两部分，它们组合起来成为dexhand。

仿照`dexhand_moveit_config对ros2_control`的处理，我们也增加了config目录，保存关节参数的配置。

这里的配置都是`基础版`，更复杂的应用我们会在dexhand_bringup中编写其单独的launch文件和相应配置。

#### 纯`.xacro`后缀文件

对于机器人的模型，我们是通过sw2urdf插件从Solidworks中导出成URDF文件。

导出的原生urdf我们保存在.xacro后缀文件中，如`left_hand_macro.xacro`和`forearm_macro.xacro`。

#### 以`.urdf.xacro`后缀文件

此类文件按需增加以下配置:

- 统一增加 world link 作为底座，这样在启动时可以复用默认的rviz配置
- 增加对rviz的颜色配置
- 增加对ros2_control的配置
- 增加对gazebo仿真的配置

## 使用

### 生成URDF文件

以`.urdf.xacro`为后缀的文件都可以通过xacro命令，生成可用的urdf文件。

```bash
xacro ./dexhand.urdf.xacro > dexhand.urdf

# 导出右手模型
xacro use_left_hand:=false ./dexhand.urdf.xacro > dexhand_rh.urdf
```

### 更新导出的机器人模型

修改对应的纯`.xacro`文件, 找到其中
```xml
<!-- pure URDF goes below -->
```
覆盖其下的xml内容即可。

### 组合新的机器人模型

复制`dexhand.urdf.xacro`文件，调整其中引用的纯xacro文件，并删减不需要的模块即可。
参考`two_dexhand.urdf.xacro`文件。

### 使用文件中的launch文件

description包中的launch文件可以直接启动机器人模型,如果需要显示其他机器人模型，可以指定`urdf_package`和`urdf_path`参数。

特别的，description.launch.py 接收xacro_args参数，允许用户设置在URDF/xacro文件中预先定义的变量值。 其中有三个参数需要注意：
- use_ros2_control:=True, 这是控制灵巧手的前提（如果这只是动画模拟不需要开启）
- use_gazebo:=False, 使用gazebo仿真，跟use_mock_dexhand最多只能设置一个为True
- use_mock_dexhand:=False, 使用ros2_control的mock_components/GenericSystem模拟硬件，跟use_gazebo最多只能设置一个为True
所以，操作真机时，xacro_args参数需要设置为：`' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false'`

```bash
# 默认使用左手，如果需要右手，可以设置xacro的use_left_hand:=false参数
ros2 launch dexhand_description description.launch.py xacro_args:='use_left_hand:=false '

# 展示其他的机器人urdf
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/two_dexhand.urdf.xacro'

# 展示跟UR机械臂结合的场景
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/ur_with_dexhand.urdf.xacro' xacro_args:='use_left_hand:=false '

# 指定xacro参数可以调整对ros2_control/gazebo的配置
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/ur_with_dexhand.urdf.xacro' xacro_args:='use_ros2_control:=true use_mock_dexhand:=true '

# 展示多个机器人，通过指定namespace参数，为/robot_description, /joint_states和ros节点加上前缀，避免冲突
# 注意rviz中要选择正确的robot_description topic，将TF Prefix填写成对应的值才能正确显示
ros2 launch dexhand_description description.launch.py namespace:='lh' use_rviz:=true
ros2 launch dexhand_description description.launch.py namespace:='rh' use_rviz:=false
```

```bash
# 使用滑动条控制灵巧手的关节
ros2 launch dexhand_description play.launch.py

# 默认使用左手，如果需要右手，可以设置xacro的use_left_hand:=false参数
ros2 launch dexhand_description play.launch.py xacro_args:='use_left_hand:=false '
```


## MuJoCo Format converting
1. fixed类型的关节，在MJCF中是没有<joint>元素的，它被省略，被<body>的层级关系隐式表达
2. floating类型的关节在MJCF中是通过<joint>元素的type="free"来表达的，但该类型只能用于根节点，允许平移和旋转6自由度的控制。
3. 实际上我们需要的是一个ball关节(不需要平移), 但直接在urdf中将type设置为ball，拖拽到Mujoco中并不能被解析
4. 所以FALJ1/FARJ1既不能套用urdf的floating，也不能自定义成ball，只能曲线救国，先改成revolute，然后导出mjcf后手动改成<joint type="ball">
