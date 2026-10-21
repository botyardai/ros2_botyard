# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

""" 
topic-pub ---/dexhand_position_controller/commands----->|
                                                        |
                                            dexhand_position_controller : ros2_control
                                                        |
                                <dexhand_hardware_interface::DexhandHardwareInterface> : RM/CM
                                                        |
                                            joint_state_broadcaster
                                                        |
rviz  <----------/joint_states--------------------------|
"""


def generate_launch_description():
    
    lh_port_name_arg = DeclareLaunchArgument(name='lh_port_name', default_value='/dev/ttyACM0', description='port name of left hand')
    rh_port_name_arg = DeclareLaunchArgument(name='rh_port_name', default_value='/dev/ttyACM1', description='port name of right hand')
    dexhand_type_arg = DeclareLaunchArgument(name='dexhand_type', default_value='gs_s1', choices=['gs_s1', 'gs_s2'])
    namespace_arg = DeclareLaunchArgument(name='namespace', default_value='two_dexhand',
                                        description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    namespace = LaunchConfiguration('namespace')
    two_dexhand_xacro_args = [' use_ros2_control:=true use_gazebo:=false ',
                              ' use_mock_dexhand:=false ',
                              ' lh_port_name:=', LaunchConfiguration('lh_port_name'),
                              ' rh_port_name:=', LaunchConfiguration('rh_port_name'),
                              ' dexhand_type:=', LaunchConfiguration('dexhand_type'), ]

    two_dexhand_rviz_config = PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'rviz', 'two_dexhand.rviz']),
    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py']),
        launch_arguments=[
            ('namespace', namespace),
            ('urdf_path', 'urdf/two_dexhand_as_individual.urdf.xacro'), # 换成两只手作为独立实体的urdf
            ('xacro_args', two_dexhand_xacro_args),
            # ('use_rviz', str(False)), # 不启用Rviz
            ('rviz_config', two_dexhand_rviz_config),
            ('use_jsp', str(False)), # 有ros2_control的joint_state_broadcaster就不能再起jsp
        ]
    )
    # 启动controller_manager 和 joint_state_broadcaster
    ros2_control = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'ros2_control.launch.py']),
        launch_arguments=[
            ('namespace', namespace),
            ('urdf_path', 'urdf/two_dexhand_as_individual.urdf.xacro'),
            ('xacro_args', two_dexhand_xacro_args),
        ]
    )

    # 该控制器的ros_parameter
    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_bringup'), 'config', 'two_dexhand_position_controller.yaml', ]
    )
    controllers = [
        "left_dexhand_position_controller",
        "right_dexhand_position_controller",
    ]
    dexhand_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=controllers + [ '--param-file', ros_parameter_file, ],
        namespace=namespace,
    )

    delay_spawner = TimerAction(
        period=3.0,  # 设置延迟时间（秒）
        actions=[dexhand_position_controller_spawner]
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    # 添加launch argument
    ld.add_action(namespace_arg)
    ld.add_action(lh_port_name_arg)
    ld.add_action(rh_port_name_arg)
    ld.add_action(dexhand_type_arg)
    # 添加各个节点
    ld.add_action(robot_description)
    ld.add_action(ros2_control)
    ld.add_action(delay_spawner)

    return ld
