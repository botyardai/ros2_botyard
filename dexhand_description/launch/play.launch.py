from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    '''
    将jsp_gui参数赋值为'True', 使用joint_state_publisher_gui, 通过滑块界面控制关节
    '''

    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py']),
        launch_arguments=[('jsp_gui', 'True'),
    ])


    ld = LaunchDescription()

    ld.add_action(robot_description)

    return ld
