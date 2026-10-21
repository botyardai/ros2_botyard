from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ''' 在Rviz中正常展示机器人模型的3个必须节点
    - robot_state_publisher: 找到并读取URDF文件，发布到/robot_description topic; 订阅 /joint_states，计算坐标系之间的变换并发布到 /tf topic
    - joint_state_publisher: 订阅/robot_description，发布/joint_states topic
    - rviz: 订阅/robot_description和/tf topic， 展示正确的机器人模型
    '''
 
    urdf_package_arg = DeclareLaunchArgument('urdf_package', default_value=TextSubstitution(text='dexhand_description'),
                                        description='The package where the robot description is located')
    urdf_path_arg = DeclareLaunchArgument('urdf_path', default_value=PathJoinSubstitution(['urdf', 'dexhand.urdf.xacro']),
                                        description='The path to the robot description relative to the urdf_package')
    xacro_args = DeclareLaunchArgument('xacro_args', default_value='',
                                        description='args for xacro when parse .xacro file')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                    description='Use simulation (Gazebo) clock if True')
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config',
                                        default_value=PathJoinSubstitution([FindPackageShare('dexhand_description'), 'rviz', 'urdf.rviz']), 
                                        description='Absolute path to rviz config file')
    use_rviz_arg = DeclareLaunchArgument(name='use_rviz', default_value='True', # 这是string而非bool，故True和true不等效
                                    description='launch rviz if True')
    use_jsp_arg = DeclareLaunchArgument(name='use_jsp', default_value='True', choices=['True', 'False'],
                                    description='Flag to disable both joint_state_publisher & joint_state_publisher_gui')
    jsp_gui_arg = DeclareLaunchArgument(name='jsp_gui', default_value='False', choices=['True', 'False'],
                                    description='Flag to enable joint_state_publisher or joint_state_publisher_gui')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='',
                                        description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    
    package_dir = FindPackageShare(LaunchConfiguration('urdf_package'))
    urdf_file = PathJoinSubstitution([package_dir, LaunchConfiguration('urdf_path')])
    robot_description_content = ParameterValue(
        # 集成多只灵巧手需要修改launch文件中的xacro_args参数, 传入相应的prefix前缀
        Command(['xacro ', urdf_file, ' ', LaunchConfiguration('xacro_args'), ]),
        value_type=str
    )


    namespace = LaunchConfiguration('namespace') # eg. "parallel/dexhand"
    # 发布/robot_description topic, 并监听/joint_states用于计算TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{ 
            'robot_description': robot_description_content, 
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 要加上'/'后缀，参考 https://github.com/ros/robot_state_publisher/blob/jazzy/src/robot_state_publisher.cpp#L270
            'frame_prefix': PythonExpression(['"" if "', namespace, '" == "" else "', namespace, '" + "/"']),
        }],
        remappings=[
            ('/robot_description', PythonExpression(['"/robot_description" if "', namespace, '" == "" else "/" + "', namespace, '" + "/robot_description"'])),
            ('/joint_states', PythonExpression(['"/joint_states" if "', namespace, '" == "" else "/" + "', namespace, '" + "/joint_states"'])),
        ]
    )
    # 监听/robot_description 和 /TF topic, 展示机器人模型
    rviz_node = Node( 
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    # 监听/robot_description topic, 根据描述发布/joint_states topic
    # Depending on use_jsp parameter, use jsp to publish joint_states(other node
    # such as joint_state_broadcaster in ros2_control can also do pub, but we only need one to do that)
    # Depending on jsp_gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        remappings=[
            ('/robot_description', PythonExpression(['"/robot_description" if "', namespace, '" == "" else "/" + "', namespace, '" + "/robot_description"'])),
            ('/joint_states', PythonExpression(['"/joint_states" if "', namespace, '" == "" else "/" + "', namespace, '" + "/joint_states"'])),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('jsp_gui'), "' == 'True'",  # 条件1: 启用gui
            " and ",                                             # 逻辑与
            "'", LaunchConfiguration('use_jsp'), "' == 'True'"  # 条件2: 允许启动jsp
        ]))
    )
    state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        remappings=[
            ('/robot_description', PythonExpression(['"/robot_description" if "', namespace, '" == "" else "/" + "', namespace, '" + "/robot_description"'])),
            ('/joint_states', PythonExpression(['"/joint_states" if "', namespace, '" == "" else "/" + "', namespace, '" + "/joint_states"'])),
        ],
        condition=UnlessCondition(PythonExpression([
            "'", LaunchConfiguration('jsp_gui'), "' == 'True'",  # 条件1: 不用gui
            " or ",                                             # 逻辑与
            "'", LaunchConfiguration('use_jsp'), "' == 'False'"  # 条件2: 允许启动jsp
        ]))
    )

    ld = LaunchDescription()

    ld.add_action(urdf_package_arg)
    ld.add_action(urdf_path_arg)
    ld.add_action(xacro_args)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(jsp_gui_arg)
    ld.add_action(use_jsp_arg)
    ld.add_action(namespace_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(state_publisher_node)
    ld.add_action(state_publisher_gui_node)

    return ld
