""" Launch gz_sim + gz_spawn_model
    copy from ros_gz_sim package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    gz_args = DeclareLaunchArgument(
        'gz_args', default_value=TextSubstitution(text='empty.sdf'), # empty world
        description='Name of the SDF world file'
    )
    gz_sim_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', LaunchConfiguration('gz_args'))],
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='empty'), # spawn model in the empty world created above
        description='the name of world into which the model will spawn'
    )
    topic_arg = DeclareLaunchArgument(
        'topic', default_value=TextSubstitution(text='robot_description'),
        description='Get XML from this topic'
    )
    entity_name_arg = DeclareLaunchArgument(
        'entity_name', default_value=TextSubstitution(text='dexhand'),
        description='Name of the entity'
    )
    ## ROS Humble没有gz_spawn_model.launch.py文件
    # spawn_model_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py'])]),
    #     launch_arguments=[('world', LaunchConfiguration('world')), # add default world name
    #                       ('topic', LaunchConfiguration('topic')), # remap arg from file:= to model_file:=
    #                       ('entity_name', LaunchConfiguration('entity_name')), # add default name
    #                       # all other args will remain the same as defined in original launch file
    # ])
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': LaunchConfiguration('world'),
                     'topic': LaunchConfiguration('topic'), # remap arg from file:= to model_file:=
                     'name':  LaunchConfiguration('entity_name'), # add deafult name
                     'x': 0.0,
                     'y': 0.0,
                     'z': 0.0,
                     'R': 0.0,
                     'P': 0.0,
                     'Y': 0.0,
                     }],
        arguments=['--topic', LaunchConfiguration('topic')],
    )

    # 在本次launch中，默认不启动Rviz
    use_rviz_arg = DeclareLaunchArgument(name='use_rviz', default_value='False',
                                    description='launch rviz if True')
    xacro_args = DeclareLaunchArgument(name='xacro_args', default_value=' use_gazebo:=true ',
                                    description='add use_gazebo:=true/false in xacro cmd when parse .xacro file')
    # 发布/robot_description, 以便spawn到gazebo中
    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'description.launch.py']),
        launch_arguments=[
            ('use_sim_time', str(True)), # pub robot_state use Gazebo clock
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('xacro_args', LaunchConfiguration('xacro_args')),
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(gz_args)
    ld.add_action(world_arg)
    ld.add_action(topic_arg)
    ld.add_action(entity_name_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(xacro_args)

    ld.add_action(robot_description)
    # Add the actions to launch all of the gz_sim + gz_spawn_model nodes
    ld.add_action(gz_sim_description)
    # ld.add_action(spawn_model_description)
    ld.add_action(spawn_model_node)

    return ld
