#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (EqualsSubstitution, IfElseSubstitution,
                                 LaunchConfiguration, PathJoinSubstitution)
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration


def _normalize_rate_hz(context, *args, **kwargs):
    """把 rate_hz 归一化成整数 Hz，写回 rate_hz_int。

    update_rate 是 int 型（controller_manager 与 ControllerInterfaceBase 都用
    auto_declare<int>），直接把 "100.0" 交给它会以类型错误 terminate。
    所以这里接受 100 / 100.0 两种写法，统一切成 int；非法值给出可读的报错。
    """
    raw = LaunchConfiguration('rate_hz').perform(context)
    try:
        value = int(float(raw))
    except ValueError:
        raise RuntimeError(
            f"rate_hz must be a number such as 100 or 100.0, got {raw!r}")
    if value <= 0:
        raise RuntimeError(f"rate_hz must be greater than zero, got {value}")
    return [SetLaunchConfiguration('rate_hz_int', str(value))]


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(name='namespace', default_value='',
                                        description='add custom namespace for all the nodes, useful for multi-robot system. no slash at the beginning & end')
    namespace = LaunchConfiguration('namespace')

    # CM / joint_state_broadcaster 的 update_rate（int；100 与 100.0 均可）
    rate_hz_arg = DeclareLaunchArgument(name='rate_hz', default_value='100',
                                        description='update rate [Hz] for controller_manager and joint_state_broadcaster; 100 and 100.0 are both accepted and normalized to an integer')
    rate_hz = LaunchConfiguration('rate_hz')
    rate_hz_int = LaunchConfiguration('rate_hz_int')

    # cap_net_raw / cap_net_admin 的获取方式，两个方案的定义与前置条件见 README
    # 「EtherCAT 连接常见问题与解决方案 - 权限与启动方式」：
    #   sudo  (方案1, 默认) 本 launch 每次启动用 `sudo + setpriv` 借能力
    #   shell (方案2) 在已带这两个能力的 shell 里启动本 launch，无需 sudo 前缀
    # 也可用环境变量 DEXHAND_CAP_GRANT 指定，便于不改命令直接切方案。
    cap_grant_arg = DeclareLaunchArgument(
        name='cap_grant',
        default_value=os.environ.get('DEXHAND_CAP_GRANT', 'sudo'),
        description='how ros2_control_node obtains cap_net_raw/cap_net_admin: '
                    'sudo (1: sudo+setpriv, needs NOPASSWD) | '
                    'shell (2: launched from a shell already holding the caps)')
    cap_grant = LaunchConfiguration('cap_grant')

    #TODO: should be a arg which could be specified by user
    ros_parameter_file = PathJoinSubstitution(
        [ FindPackageShare('dexhand_description'), 'config', 'ros2_controllers.yaml']
    )

    # ros2_control_node 跑 SOEM EtherCAT 需要两个 capability：
    #   cap_net_raw   → 建 AF_PACKET raw socket（SOEM nicdrv.c: socket(PF_PACKET, SOCK_RAW, ...)）
    #   cap_net_admin → 置 IFF_PROMISC / 改网卡 flags（SOEM nicdrv.c: ioctl(SIOCSIFFLAGS)）
    #
    # 方案1（cap_grant:=sudo，默认）用 setpriv 只借这两个能力，随后立刻降回当前用户：
    #   * 进程身份仍是当前用户，与 spawner / RViz 同用户，不存在跨用户 DDS 问题；
    #   * 不触发 secure-exec（AT_SECURE=0），无需改 ld.so.conf（但 LD_LIBRARY_PATH
    #     仍会被 sudo 清掉，见下方 env 注入）。
    # 前提：当前用户 sudo 免密（NOPASSWD）。否则 launch 子进程没有 tty，会卡在密码提示。
    # 注意：本 launch 需以普通用户启动；若整条 launch 用 sudo 跑，降权会失效。
    # sudo 会强制清掉 LD_LIBRARY_PATH（它在 sudo 的默认删除列表里，`-E` 和
    # `--preserve-env=LD_LIBRARY_PATH` 都保不住），而 ros2_control_node 没有 RUNPATH、
    # ROS 库也不在 ld.so.cache 中，所以必须用 env 把当前值作为参数显式传进去，
    # 否则会报 libbackward.so: cannot open shared object file。
    #
    # 方案2 由带能力的 shell 预先备好能力（ambient），这里就不再加前缀；
    # 对 launch 而言只是「当前进程已带能力」。
    _ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    _privileged_prefix = [
        'sudo -E /usr/bin/setpriv '
        f'--reuid={os.getuid()} --regid={os.getgid()} --init-groups '
        '--inh-caps=+net_raw,+net_admin --ambient-caps=+net_raw,+net_admin '
        f'/usr/bin/env LD_LIBRARY_PATH={_ld_library_path} '
    ]
    _cap_grant_prefix = IfElseSubstitution(
        EqualsSubstitution(cap_grant, 'shell'),
        if_value='',  # 方案2：能力已在当前进程里，不加前缀
        else_value=_privileged_prefix,  # 方案1
    )

    # 启动 ros2_control_node 节点，该节点加载控制器管理器及硬件接口
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # rate_hz 经参数覆盖写进 update_rate；controller_manager 会把自己的 --params-file
        # 透传给每个控制器，故该覆盖同时命中 joint_state_broadcaster.update_rate
        # （joint_state_broadcaster.publish_rate 在 Jazzy 上不存在，不写）。
        parameters=[ros_parameter_file, {'update_rate': rate_hz_int}],
        # 只有该节点需要 raw socket 权限
        prefix=_cap_grant_prefix,
        remappings=[
            # default listen to 'robot_description' topic, which respect to namespace
            #  aka. /${namespace}/robot_description, not /${namespace}/${node_name}/robot_description
            # ("robot_description", "/robot_description"), # fixed to global topic
            # ("robot_description", "~/robot_description"), # adjust with node_name
            # we just do as default, which respect to namespace automaticly
        ],
        # arguments=['--ros-args', '--log-level', 'debug'],
        output="both",
        namespace=namespace,
    )
    
    # 这其实也是一个controller，将各关节的state发布到/joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', ],
        # respect to namespace by default. But if you want to connect to other namespace controller-manager, you can point out like below
        # arguments=['joint_state_broadcaster', '--controller-manager', ['/', ${other-namespace}, '/controller_manager']],
        output='screen',
        namespace=namespace,
    )
    

    ld = LaunchDescription()
    # 添加launch argument
    ld.add_action(namespace_arg)
    ld.add_action(rate_hz_arg)
    ld.add_action(cap_grant_arg)
    # rate_hz 须先归一化成整数，且必须在 ros2_control_node 之前执行
    ld.add_action(OpaqueFunction(function=_normalize_rate_hz))
    # 添加各个节点
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
