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
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
topic-pub --------/dexhand_controller/commands--------->|
                                                        |
                                            dexhand_controller : ros2_control
                                                        |
                                <dexhand_hardware_interface::DexhandHardwareInterface> : RM/CM
                                                        |
                                            joint_state_broadcaster
                                                        |
rviz  <----------/joint_states--------------------------|
"""


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Custom namespace for multi-robot systems')
    namespace = LaunchConfiguration('namespace')
    xacro_args = DeclareLaunchArgument(
        'xacro_args',
        default_value=(
            ' use_ros2_control:=true use_gazebo:=false '
            'use_mock_dexhand:=false'),
        description='Arguments used while parsing the xacro file')
    dexhand_type_arg = DeclareLaunchArgument(
        'dexhand_type', default_value='gs_s1', choices=['gs_s1', 'gs_s2'])
    versioned_xacro_args = [
        LaunchConfiguration('xacro_args'),
        ' dexhand_type:=',
        LaunchConfiguration('dexhand_type'),
    ]
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='True')
    # 透传给 ros2_control.launch.py：cap_net_raw / cap_net_admin 的获取方式
    # （sudo|shell，两个方案见 README「权限与启动方式」）
    cap_grant_arg = DeclareLaunchArgument(
        name='cap_grant',
        default_value=os.environ.get('DEXHAND_CAP_GRANT', 'sudo'),
        description='how ros2_control_node obtains cap_net_raw/cap_net_admin: '
                    'sudo (1: sudo+setpriv, needs NOPASSWD) | '
                    'shell (2: launched from a shell already holding the caps)')

    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('dexhand_description'),
            'launch',
            'description.launch.py',
        ]),
        launch_arguments=[
            ('namespace', namespace),
            ('xacro_args', versioned_xacro_args),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('use_jsp', str(False)),  # joint_state_broadcaster owns joint_states
        ]
    )
    # 启动controller_manager 和 joint_state_broadcaster
    ros2_control = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('dexhand_description'),
            'launch',
            'ros2_control.launch.py',
        ]),
        launch_arguments=[
            ('namespace', namespace),
            ('xacro_args', versioned_xacro_args),
            ('cap_grant', LaunchConfiguration('cap_grant')),
        ]
    )

    # 该控制器的ros_parameter
    ros_parameter_file = PathJoinSubstitution(
        [FindPackageShare('dexhand_bringup'), 'config', 'dexhand_controller.yaml']
    )
    dexhand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dexhand_controller',
            '--param-file',
            ros_parameter_file,
        ],
        namespace=namespace,
    )

    delay_spawner = TimerAction(
        period=3.0,  # 设置延迟时间（秒）
        actions=[dexhand_controller_spawner]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # 添加launch argument
    ld.add_action(namespace_arg)
    ld.add_action(xacro_args)
    ld.add_action(dexhand_type_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(cap_grant_arg)
    # 添加各个节点
    ld.add_action(robot_description)
    ld.add_action(ros2_control)
    ld.add_action(delay_spawner)

    return ld
