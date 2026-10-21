from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    xacro_args = DeclareLaunchArgument(
        'xacro_args',
        default_value=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false',
        description='args for xacro when parse .xacro file'
    )

    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py']),
        launch_arguments=[
            ('xacro_args', LaunchConfiguration('xacro_args')), # 用真机/启用ros2_control
            ('use_rviz', str(True)), # 启用Rviz
            ('rviz_config', PathJoinSubstitution([FindPackageShare('dexhand_moveit_config'), 'config', 'move_group.rviz'])),
            ('use_jsp', str(False)), # 有ros2_control的joint_state_broadcaster就不能再起jsp
        ]
    )

    # 启动controller_manager 和 joint_state_broadcaster
    ros2_control = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'ros2_control.launch.py']),
        launch_arguments=[
            ('xacro_args', LaunchConfiguration('xacro_args')),
        ],
    )
    # dexhand_controller的ros_parameter
    use_joint_trajectory_controller_arg = DeclareLaunchArgument(name='use_joint_trajectory_controller', default_value='False', choices=['True', 'False'],
                                    description='use joint_trajectory_controller if True')
    dexhand_jtc_parameter_file = PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'config', 'dexhand_joint_trajectory_controller.yaml',])
    dexhand_jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dexhand_joint_trajectory_controller',
            '--param-file',
            dexhand_jtc_parameter_file,
        ],
        condition=IfCondition(LaunchConfiguration('use_joint_trajectory_controller')),
    )
    # dexhand_position_controller的ros_parameter
    use_position_controller_arg = DeclareLaunchArgument(name='use_position_controller', default_value='False', choices=['True', 'False'],
                                    description='use position_controller if True')
    dexhand_position_controller_parameter_file = PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'config', 'dexhand_position_controller.yaml',])
    dexhand_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dexhand_position_controller',
            '--param-file',
            dexhand_position_controller_parameter_file,
        ],
        condition=IfCondition(LaunchConfiguration('use_position_controller')),
    )

    # moveit相关的配置
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    semantic_robot_description_file = os.path.join(get_package_share_directory("dexhand_moveit_config"), "config", "dexhand.srdf")
    kinematics_file = os.path.join(get_package_share_directory("dexhand_moveit_config"), "config", "kinematics.yaml")
    # jointlimit_file = os.path.join(get_package_share_directory("dexhand_moveit_config"), "config", "joint_limits.yaml")
    moveit_config_dict = (
        MoveItConfigsBuilder(robot_name="dexhand", package_name="dexhand_moveit_config")
        # .robot_description_semantic(semantic_robot_description_file)
        # .robot_description_kinematics(kinematics_file)
        # .joint_limits(jointlimit_file)
        .planning_scene_monitor(publish_robot_description_semantic=True) # rviz need this
        .to_moveit_configs()
        .to_dict()
    )
    # We do not pass the robot_description parameter directly to move_group
    # Then the move_group node will obtain it from appropriate topic
    moveit_config_dict.pop("robot_description")
    # Start the actual move_group node/action server, 处理rviz的moveit插件发出的指令
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            trajectory_execution,
        ],
        # namespace='',
        # remappings=[('/robot_description', '/dexhand/robot_description')],
        # 添加日志级别参数，设置为debug
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    delay_action = TimerAction(
        period=5.0,  # 设置延迟时间（秒）
        actions=[dexhand_jtc_spawner, dexhand_position_controller_spawner, move_group_node]
    )
    '''
    after_move_group_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=move_group_node,
            on_exit=[dexhand_jtc_spawner, dexhand_position_controller_spawner],
        )
    )
    '''

    # Create the launch description and populate
    ld = LaunchDescription()
    # add launch arguments
    ld.add_action(xacro_args)
    ld.add_action(use_joint_trajectory_controller_arg)
    ld.add_action(use_position_controller_arg)
    # add nodes and includes
    ld.add_action(robot_description)
    ld.add_action(ros2_control)
    ld.add_action(delay_action)

    return ld
