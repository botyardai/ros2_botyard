from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

""" 
topic-pub -----/joint_position_cmd-------> |
                                           |
                                    ros_gz_bridge  <---->  gazebo
                                           |
rviz  <--------/joint_states---------------|
"""

def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    bridge_name_arg = DeclareLaunchArgument(
        'bridge_name',
        default_value='botyard_gz_bridge',
        description='Name of the bridge'
    )

    config_file = LaunchConfiguration('config_file') # topic configuration file used by ros bridge 
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([FindPackageShare('dexhand_bringup'), 'config', 'bridge_topic.yaml']),
        description='YAML config file'
    )
            
    container_name = LaunchConfiguration('container_name')
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    create_own_container = LaunchConfiguration('create_own_container')
    create_own_container_arg = DeclareLaunchArgument(
        'create_own_container',
        default_value='True',
        description='Whether we should start a ROS container when using composition.',
    )

    namespace = LaunchConfiguration('namespace') # use by ros bridge
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    use_composition = LaunchConfiguration('use_composition')
    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    use_respawn = LaunchConfiguration('use_respawn')
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    bridge_log_level = LaunchConfiguration('bridge_log_level')
    bridge_log_level_arg = DeclareLaunchArgument(
        'bridge_log_level', default_value='info', description='Bridge log level'
    )
    # ros_gz_bridge负责打通ROS <---> Gazebo的数据通道
    bridge_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_bridge'), 'launch', 'ros_gz_bridge.launch.py'])]),
        launch_arguments=[('bridge_name', bridge_name),
                          ('config_file', config_file),
                          ('container_name', container_name),
                          ('namespace', namespace),
                          # ('create_own_container', str(False)),
                          ('create_own_container', create_own_container),
                          ('use_composition', use_composition),
                          ('use_respawn', use_respawn),
                          ('bridge_log_level', bridge_log_level), ])

    # gazebo从/robot_description topic获取urdf内容，spawn到gazebo world
    gazebo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'gazebo.launch.py'])]),
        launch_arguments=[
            ('gz_args', ' -r -v 1 empty.sdf'), # 让Gazebo开机启动仿真 -r to run simlulation on start and -v log in verbose use level 4
            ('xacro_args', ' use_ros2_control:=true use_gazebo:=true use_mock_dexhand:=false'),
            ('use_rviz', str(True)), # 启用Rviz
            ('use_jsp', str(False)), # 由bridge直接透传就不能再起jsp
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(bridge_name_arg)
    ld.add_action(config_file_arg)
    ld.add_action(container_name_arg)
    ld.add_action(create_own_container_arg)
    ld.add_action(namespace_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(use_respawn_arg)
    ld.add_action(bridge_log_level_arg)

    ld.add_action(bridge_description)
    ld.add_action(gazebo_description)

    return ld
