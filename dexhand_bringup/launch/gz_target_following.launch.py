from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import load_xacro
from pathlib import Path

'''
topic-pub/juexian-app -------/dexhand/target_following--------------> |
                                                                      |
                                                        my_custom_target_following(compute a valide target POSE)
                                                                      |
                                                        move_group(IK, OMPL, planning_pipeline)
                                                                      |
                                                        dexhand_controller(joint_trajectory_controller)
                                                                      |
                                            <gz_ros2_control::GazeboSimROS2ControlPlugin> : gazebo
                                                                      |
                                                        joint_state_broadcaster(ros2_control)
                                                                      |
rviz  <----------------------/joint_states--------------------------- |
'''

def generate_launch_description():

    # 新增rosbridge启动
    rosbridge_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])]
        ),
        launch_arguments=[
            ('port', '9090')
        ],
    )

    gz_moveit_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'launch', 'gz_moveit.launch.py'])]),
        launch_arguments=[
            ('use_target_following', str(True)), # 启动 target following
        ],
    )

    # 设置启动依赖
    timer_action = TimerAction(
        period=5.0,  # 设置延迟时间（秒）
        actions=[rosbridge_node]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(gz_moveit_description)
    ld.add_action(timer_action)

    return ld
