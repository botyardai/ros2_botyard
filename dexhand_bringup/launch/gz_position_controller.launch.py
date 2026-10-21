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
topic-pub --- /dexhand_position_controller/commands --->|
                                                        |
                                            position_controller : ros2_control
                                                        |
                                <gz_ros2_control::GazeboSimROS2ControlPlugin> : CM/RM
                                                        |
                                            joint_state_broadcaster
                                                        |
rviz  <----------/joint_states--------------------------|
"""


def generate_launch_description():

    # 这其实也是一个controller，通过ros2_control框架读取各关节的state发布到/joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[ 'joint_state_broadcaster' ],
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
    )

    timer_action = TimerAction(
        period=3.0,  # 设置延迟时间（秒）
        actions=[joint_state_broadcaster_spawner]
    )

    # 设置启动依赖
    reh = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dexhand_position_controller_spawner],
        )
    )

    # Bridge, 将Gazebo时钟发布到ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # gazebo从/robot_description topic获取urdf内容，spawn到gazebo world
    # gz_ros2_control本身就是controller_manager
    gazebo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'gazebo.launch.py'])]),
        launch_arguments=[
            ('gz_args', ' -r -v 1 empty.sdf'), # 让Gazebo开机启动仿真 -r to run simlulation on start and -v log in verbose use level 4
            ('xacro_args', ' use_ros2_control:=true use_gazebo:=true use_mock_dexhand:=false'),
            # ('use_rviz', str(True)), # 启用Rviz
            ('use_jsp', str(False)), # 有ros2_control的joint_state_broadcaster就不能再起jsp
        ]
    )
    

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(gazebo_description)
    ld.add_action(bridge)
    ld.add_action(timer_action)
    ld.add_action(reh)

    return ld
