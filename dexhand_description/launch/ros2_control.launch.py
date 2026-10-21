#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(name='namespace', default_value='',
                                        description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    namespace = LaunchConfiguration('namespace')

    #TODO: should be a arg which could be specified by user
    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_description'), 'config', 'ros2_controllers.yaml']
    )
    # 启动 ros2_control_node 节点，该节点加载控制器管理器及硬件接口
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros_parameter_file],
        remappings=[
            # default listen to 'robot_description' topic, which respect to namespace
            #  aka. /${namespace}/robot_description, not /${namespace}/${node_name}/robot_description
            # ("robot_description", "/robot_description"), # fixed to global topic
            # ("robot_description", "~/robot_description"), # adjust with node_name
            # we just do as default, which respect to namespace automaticly
        ],
        # arguments=['--ros-args', '--log-level', 'debug'],
        output="both",
        namespace=namespace,
    )
    
    # 这其实也是一个controller，将各关节的state发布到/joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', ],
        # respect to namespace by default. But if you want to connect to other namespace controller-manager, you can point out like below
        # arguments=['joint_state_broadcaster', '--controller-manager', ['/', ${other-namespace}, '/controller_manager']],
        output='screen',
        namespace=namespace,
    )
    

    ld = LaunchDescription()
    # 添加launch argument
    ld.add_action(namespace_arg)
    # 添加各个节点
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
