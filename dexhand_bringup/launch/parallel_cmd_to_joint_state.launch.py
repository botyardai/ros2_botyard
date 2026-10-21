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
topic-pub    -------------------${cmd_topic_arg}------------------->|
                                                                    |
                                                   <${namespace}::parallel_cmd_to_joint_state>
                                                                    |
robot_state_publisher  <--------${joint_state_topic_arg}------------|
            |
            |-------------------${namespace}/robot_description---> rviz2
"""


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument('namespace', default_value='parallel/dexhand',
                                        description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    cmd_topic_arg = DeclareLaunchArgument('cmd_topic', default_value='/dexhand_position_controller/commands',
                                        description='read std_msg/Float64MultiArray from cmd_topic and convert to sensor_msgs/JointState')
    joint_state_topic_arg = DeclareLaunchArgument('joint_state_topic', default_value='/parallel/dexhand/joint_states',
                                        description='read std_msg/Float64MultiArray from cmd_topic and convert to sensor_msgs/JointState')
    
    dexhand_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py'])]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_rviz', str(True)), # 启用Rviz
            ('use_jsp', str(False)),
        ]
    )
    
    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_bringup'), 'config', 'parallel_cmd_to_joint_state.yaml']
    )
    parallel_cmd_to_joint_state_node = Node(
        package='dexhand_bringup',
        executable='parallel_cmd_to_joint_state',
        namespace=LaunchConfiguration('namespace'),
        parameters=[ros_parameter_file],
        # arguments=[
        #     '--param-file',
        #     ros_parameter_file,
        # ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(namespace_arg)
    ld.add_action(cmd_topic_arg)
    ld.add_action(joint_state_topic_arg)

    ld.add_action(dexhand_description)
    ld.add_action(parallel_cmd_to_joint_state_node)

    return ld
