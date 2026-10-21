from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import load_xacro
from pathlib import Path

'''
motion_planning(rviz plugin) ---moveit_simpler_controller_manager---> |
                                                                      |
                                                        (IK, OMPL, planning_pipeline)
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

     # 这其实也是一个controller，将各关节的state发布到/joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[ 'joint_state_broadcaster' ],
    )

    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_moveit_config'), 'config', 'ros2_controllers.yaml', ]
    )
    dexhand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dexhand_controller',
            '--param-file',
            ros_parameter_file,
        ],
    )

    # Bridge, 将Gazebo时钟发布到ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    xacro_args_str = ' use_gazebo:=true use_ros2_control:=true use_mock_dexhand:=false'
    # gazebo从/robot_description topic获取urdf内容，spawn到gazebo world
    gazebo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'gazebo.launch.py'])]),
        launch_arguments=[
            ('gz_args', ' -r -v 1 empty.sdf'), # 让Gazebo开机启动仿真 -r to run simlulation on start and -v log in verbose use level 4
            ('xacro_args', xacro_args_str), # 启用ros2_control 
            ('use_rviz', str(False)), # 不启用Rviz
            ('use_jsp', str(False)), # 有ros2_control的joint_state_broadcaster就不能再起jsp
        ]
    )

    # moveit相关的配置文件
    moveit_config_dict = MoveItConfigsBuilder("dexhand", package_name="dexhand_moveit_config").to_dict()
    # 使用同样的urdf文件（ <ros2_control>tag保持一致 ）
    moveit_config_dict['robot_description'] = load_xacro(
      Path(get_package_share_directory('dexhand_description') + '/urdf/dexhand.urdf.xacro'),
      dict( arg.split(":=") for arg in xacro_args_str.split(" ") if arg ),
    )

    # 启动定制化的 RViz，增加了moveit相关插件，所以要传入相关参数文件
    rviz_config_file = PathJoinSubstitution([FindPackageShare('dexhand_moveit_config'), 'config', 'moveit.rviz'])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config_dict,
            {'use_sim_time': True}  # 使用gazebo bridge透传的/clock时钟主题
        ],
    )

    # Start the actual move_group node/action server, 处理rviz的moveit插件发出的指令
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            {'use_sim_time': True}  # 使用gazebo bridge透传的/clock时钟主题
        ],
    )

    use_target_following_arg = DeclareLaunchArgument(name='use_target_following', default_value='False', choices=['True', 'False'],
                                    description='enable custom target following node if True')
    # a move_group client(代替Rviz的moveit插件gui)
    target_following_node = Node(
        name="my_custom_target_following",
        package="dexhand_moveit_config",
        executable="my_custom_target_following",
        output="screen",
        parameters=[
            moveit_config_dict,
            {'use_sim_time': True}  # 使用gazebo bridge透传的/clock时钟主题
        ],
        condition=IfCondition(LaunchConfiguration('use_target_following')),
    )

    # 设置启动依赖
    timer_action = TimerAction(
        period=5.0,  # 设置延迟时间（秒）
        actions=[joint_state_broadcaster_spawner, dexhand_controller_spawner, target_following_node]
    )
    reh = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dexhand_controller_spawner,
            on_exit=[rviz_node, move_group_node],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # export args
    ld.add_action(use_target_following_arg)

    ld.add_action(gazebo_description)
    ld.add_action(bridge)
    ld.add_action(timer_action)
    ld.add_action(reh)

    return ld
