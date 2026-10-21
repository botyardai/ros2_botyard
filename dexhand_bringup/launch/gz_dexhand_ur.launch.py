from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    '''
    启动gazebo, 并加载dexhand_description/launch/description.launch.py
    '''
    # UR机械臂配置
    ur_type_arg = DeclareLaunchArgument(name='ur_type', default_value='ur16e', 
                                    choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 
                                             'ur10e', 'ur16e', 'ur20'],
                                    description='Type/series of used UR robot')
    
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value='',
                                    description='Name of the robot')
    
    tf_prefix_arg = DeclareLaunchArgument(name='tf_prefix', default_value='',
                                    description='Prefix for TF frames')

    urdf_package_arg = DeclareLaunchArgument('urdf_package', default_value=TextSubstitution(text='dexhand_description'),
                                        description='The package where the robot description is located')
    urdf_path_arg = DeclareLaunchArgument('urdf_path', default_value=PathJoinSubstitution(['urdf', 'dexhandUr.urdf.xacro']),
                                        description='The path to the robot description relative to the package root')
    # 加载dexhand_description/launch/description.launch.py
    # 传入参数：urdf_package_arg，urdf_path_arg
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('dexhand_description'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments=[
            ('ur_type', LaunchConfiguration('ur_type')),
            ('robot_name', LaunchConfiguration('robot_name')),
            ('tf_prefix', LaunchConfiguration('tf_prefix')),
            ('urdf_package', LaunchConfiguration('urdf_package')),
            ('urdf_path', LaunchConfiguration('urdf_path')),
            ('use_sim_time', str(True)), # pub robot_state use Gazebo clock
            ('use_ros2_control', 'True'), # 启动ros2_control
            ('use_gazebo', 'True'), # 启动gazebo
            ('use_rviz', 'True'), # 启动rviz
            ('load_left', 'true'),
            ('load_right', 'true'),
            ('xacro_args', ' use_ros2_control:=true use_gazebo:=true use_mock_dexhand:=false'), # 加载gazebo相关的xacro参数
            ('use_jsp', 'True'), # 启动joint_state_publisher
            ('jsp_gui', 'False'), # 不启动joint_state_publisher_gui
            ('jsp_topic', '/joint_states'), 
        ]
    )


    ld = LaunchDescription()

    ld.add_action(ur_type_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(tf_prefix_arg)
    ld.add_action(urdf_package_arg)
    ld.add_action(urdf_path_arg)
    ld.add_action(robot_description)
    return ld