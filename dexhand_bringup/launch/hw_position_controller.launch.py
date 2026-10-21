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

    namespace_arg = DeclareLaunchArgument(name='namespace', default_value='', description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    namespace = LaunchConfiguration('namespace')
    xacro_args = DeclareLaunchArgument('xacro_args', default_value=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false',
                                        description='args for xacro when parse .xacro file')
    
    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py']),
        launch_arguments=[
            ('namespace', namespace),
            ('xacro_args', LaunchConfiguration('xacro_args')),
            # ('use_rviz', str(True)), # 启用Rviz
            ('use_jsp', str(False)), # 有ros2_control的joint_state_broadcaster就不能再起jsp
        ]
    )
    # 启动controller_manager 和 joint_state_broadcaster
    ros2_control = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'ros2_control.launch.py']),
        launch_arguments=[
            ('namespace', namespace),
            ('xacro_args', LaunchConfiguration('xacro_args')),
        ]
    )

    # 该控制器的ros_parameter
    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_bringup'), 'config', 'dexhand_position_controller.yaml', ]
    )
    dexhand_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dexhand_position_controller',
            '--param-file',
            ros_parameter_file,
        ],
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
    ld.add_action(xacro_args)
    # 添加各个节点
    ld.add_action(robot_description)
    ld.add_action(ros2_control)
    ld.add_action(delay_spawner)

    return ld
