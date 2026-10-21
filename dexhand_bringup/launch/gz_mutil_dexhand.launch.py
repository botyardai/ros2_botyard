import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_hand_launch_descriptions(context, num_hands):
    hand_launch_descriptions = []
    try:
        num_hands_value = int(num_hands.perform(context))
    except ValueError:
        print("Invalid value for num_hands, using 0 as default.")
        num_hands_value = 0
    for i in range(num_hands_value):
        # 包含 description.launch.py 发布机器人描述信息
        robot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('dexhand_description'),
                    'launch',
                    'description.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_sim_time', 'True'),
                ('robot_name', f'dexhand_{i}'),
                ('xacro_args', ' use_ros2_control:=true use_gazebo:=true load_left:=true load_right:=true'),
            ]
        )

        # 包含 gazebo.launch.py 的模型生成部分，复用 Gazebo 环境
        spawn_model = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('dexhand_description'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments=[
                ('world', LaunchConfiguration('world').perform(context)),
                ('robot_name', f'dexhand_{i}'),
                ('x', str(float(i * 2))),
                ('y', '0.0'),
                ('z', '0.0'),
                ('use_gazebo', 'False'),
                ('use_rviz', 'False')
            ]
        )

        # 每加载一个模型延迟 5 秒
        delayed_spawn = TimerAction(
            period=5.0 * i,
            actions=[robot_description, spawn_model]
        )

        hand_launch_descriptions.append(delayed_spawn)
    return hand_launch_descriptions


def generate_launch_description():
    pkg_dexhand_description = get_package_share_directory('dexhand_description')

    # 声明启动参数(个或者组)
    num_hands = LaunchConfiguration('num_hands')
    num_hands_arg = DeclareLaunchArgument(
        'num_hands',
        default_value='1',
        description='Number of dexhands to spawn in Gazebo'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty',
        description='the name of world into which the model will spawn'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dexhand_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[
            ('use_gazebo', 'True'),
            ('use_rviz', 'False'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('dexhand_description'),
                'rviz',
                'urdf.rviz'
            ])
        ],
    )

    hand_launch_actions = OpaqueFunction(
        function=generate_hand_launch_descriptions,
        args=[num_hands]
    )

    return LaunchDescription([
        num_hands_arg,
        world_arg,
        gazebo_launch,
        rviz_node,
        hand_launch_actions
    ])
