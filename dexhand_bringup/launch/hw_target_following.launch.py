from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

'''
topic-pub/juexian-app -------/dexhand/target_following--------------> |
                                                                      |
                                                        my_custom_target_following(compute a valide target POSE)
                                                                      |
                                                        move_group(IK, OMPL, planning_pipeline)
                                                                      |
                                                        dexhand_controller(joint_trajectory_controller)
                                                                      |
                                            <dexhand_hardware_interface::DexhandHardwareInterface> : RM/CM
                                                                      |
                                                        joint_state_broadcaster(ros2_control)
                                                                      |
rviz  <----------------------/joint_states--------------------------- |
'''

def generate_launch_description():
    xacro_args = DeclareLaunchArgument(
        'xacro_args',
        default_value=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false',
        description='args for xacro when parse .xacro file'
    )

    hw_moveit_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'launch', 'hw_moveit.launch.py'])]),
        launch_arguments=[
            ('xacro_args', LaunchConfiguration('xacro_args')),
            ('use_position_controller', str(True)), # 启动 position_controller
            ('use_joint_trajectory_controller', str(False)),
        ]
    )

    # 新增rosbridge启动
    rosbridge_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])]
        ),
        launch_arguments=[
            ('port', '9090')
        ],
    )

    # moveit相关的配置
    moveit_config_dict = (
        MoveItConfigsBuilder(robot_name="dexhand", package_name="dexhand_moveit_config")
        .to_moveit_configs()
        .to_dict()
    )
    # We do not pass the robot_description parameter directly to move_group
    # Then the move_group node will obtain it from appropriate topic
    moveit_config_dict.pop("robot_description")
    target_following_node = Node(
        name="my_custom_target_following",
        package="dexhand_moveit_config",
        executable="my_custom_target_following",
        output="screen",
        parameters=[
            moveit_config_dict,
        ],
    )

    # 设置启动依赖
    timer_action = TimerAction(
        period=5.0,  # 设置延迟时间（秒）
        actions=[rosbridge_node, target_following_node]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(xacro_args)
    ld.add_action(hw_moveit_description)
    ld.add_action(timer_action)

    return ld
