# dexhand_description

## Overview

This repository is mainly used for managing the modeling of the dexterous hand, such as URDF, Xacro, Mesh, .rviz files, etc.

## Design Concept

The dexterous hand is split into two parts: hand and forearm, which are combined to form dexhand.

Following the approach of `dexhand_moveit_config` for ros2_control, we also added a config directory to store joint parameter configurations.

The configurations here are the "basic version". More complex applications will have their own launch files and corresponding configurations written in dexhand_bringup.

#### Pure `.xacro` Suffix Files

For the robot model, we export URDF files from Solidworks using the sw2urdf plugin.

The original exported urdf files are saved as .xacro files, such as `left_hand_macro.xacro` and `forearm_macro.xacro`.

#### `.urdf.xacro` Suffix Files

These files add the following configurations as needed:

- Add a unified world link as the base, so the default rviz configuration can be reused at startup
- Add color configuration for rviz
- Add configuration for ros2_control
- Add configuration for gazebo simulation

## Usage

### Generate URDF Files

Files with the `.urdf.xacro` suffix can be converted to usable urdf files using the xacro command.

```bash
xacro ./dexhand.urdf.xacro > dexhand.urdf

# Export right hand model
xacro use_left_hand:=false ./dexhand.urdf.xacro > dexhand_rh.urdf
```

### Update Exported Robot Model

Modify the corresponding pure `.xacro` file, find
```xml
<!-- pure URDF goes below -->
```
and overwrite the xml content below it.

### Combine New Robot Models

Copy the `dexhand.urdf.xacro` file, adjust the referenced pure xacro files, and remove unnecessary modules as needed. Refer to the `two_dexhand.urdf.xacro` file.

### Use Launch Files in the Package

The launch files in the description package can directly start the robot model. If you need to display other robot models, you can specify the `urdf_package` and `urdf_path` parameters.

```bash
# By default, the left hand is used. If you need the right hand, set the xacro parameter use_left_hand:=false
ros2 launch dexhand_description description.launch.py xacro_args:='use_left_hand:=false '

# Display other robot urdf
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/two_dexhand.urdf.xacro'

# Display the scenario combined with the UR robotic arm
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/ur_with_dexhand.urdf.xacro' xacro_args:='use_left_hand:=false '

# Specify xacro parameters to adjust ros2_control/gazebo configuration
ros2 launch dexhand_description description.launch.py urdf_path:='urdf/ur_with_dexhand.urdf.xacro' xacro_args:='use_ros2_control:=true use_mock_dexhand:=true '

# Display multiple robots by specifying the namespace parameter, adding a prefix to /robot_description, /joint_states, and ros nodes to avoid conflicts
# Note: In rviz, select the correct robot_description topic and set the TF Prefix to the corresponding value for correct display
ros2 launch dexhand_description description.launch.py namespace:='lh' use_rviz:=true
ros2 launch dexhand_description description.launch.py namespace:='rh' use_rviz:=false
```

```bash
# Use sliders to control the joints of the dexterous hand
ros2 launch dexhand_description play.launch.py

# By default, the left hand is used. If you need the right hand, set the xacro parameter use_left_hand:=false
ros2 launch dexhand_description play.launch.py xacro_args:='use_left_hand:=false '
```

## MuJoCo Format Converting
1. Joints of type fixed do not have a <joint> element in MJCF; they are omitted and implicitly expressed by the hierarchy of <body>.
2. Joints of type floating are represented in MJCF by a <joint> element with type="free", but this type can only be used for the root node, allowing 6-DOF translation and rotation control.
3. In practice, what we need is a ball joint (without translation), but directly setting the type to ball in urdf and dragging it into Mujoco cannot be parsed.
4. Therefore, FALJ1/FARJ1 cannot use urdf's floating nor custom ball; the workaround is to first change them to revolute, then manually change to <joint type="ball"> in the exported mjcf.
