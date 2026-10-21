# Dexhand EtherCAT 使用与故障排查

本文汇总灵巧手 **EtherCAT** 通信后端的启动方式、数据面、链路参数、故障自愈与常见问题。
内容从 [README.md](README.md) 抽离并补充，启动命令的通用约定见 README 主文档。
首次在新机器上使用，请先完成第 2 节「前置准备」。

## 1. 架构与职责划分

真机走 `communicate:=EtherCAT` 时，EtherCAT 链路由 ros2_control 的
`DexhandHardwareInterface` 负责，应用层只通过话题 / 服务 / action 与它交互，
不需要也不应该直接操作网卡或 SOEM：

```text
/dexhand_controller/commands (topic)      /dexhand_controller/do/gesture (action)
              |                                        |
              v                                        v
        dexhand_controller  (ros2_control controller, 应用层唯一入口)
              |
              v
  DexhandHardwareInterface  (SystemInterface, communicate:=EtherCAT)
     io_thread：execCmd* 下发 / getDofsAndTactiles 回读；watchdog：故障重建
              |  用户态 PDO / SDO
              v
  dexhand_driver  (SOEM，预编译库 /usr/local/lib/libdexhand_driver.so)
              |
              v
        灵巧手 EtherCAT 从站
```

要点：

- EtherCAT 后端选择的判定点在 `init_driver_()`
  （[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L310-L383)）：
  `communicate` 统一转小写后匹配 `canfd` / `ethercat` / `rs485`，其它值报
  `unknow communication` 并让 hardware configure 失败。
- `communicate` 由 URDF/xacro 注入到 `<ros2_control>` 硬件段的 `<param name="communicate">`
  （[dexhand.ros2_control.xacro](../dexhand_description/config/dexhand.ros2_control.xacro#L149-L159)），
  **不是**控制器 yaml 里的配置项（`ros2_controllers.yaml` / `dexhand_controller.yaml` 都不含 EtherCAT 参数）。
- 默认值是 `communicate:=RS485`
  （[dexhand.ros2_control.xacro](../dexhand_description/config/dexhand.ros2_control.xacro#L118-L119)），
  因此必须有显式 `communicate:=EtherCAT` 才走 EtherCAT。

## 2. 前置准备：配置免密 sudoers（推荐）

EtherCAT 后端（SOEM）需要两个 Linux capability：`CAP_NET_RAW`（建 `AF_PACKET/SOCK_RAW`）
与 `CAP_NET_ADMIN`（置 `IFF_PROMISC`、改网卡 flags）。默认的 `cap_grant:=sudo` 方案由 launch
用 `sudo -E /usr/bin/setpriv ...` 临时借出这两个能力，所以要求当前用户能**免密**调用 `setpriv`。
（两个方案的完整对比见第 8 节「权限与启动方式」。）

仓库自带 [99_ethercat_cap](../dexhand_description/config/99_ethercat_cap)，
内容为 `%sudo ALL=(root) NOPASSWD: SETENV: /usr/bin/setpriv`：

| 要素 | 作用 |
| --- | --- |
| `NOPASSWD` | launch 子进程没有 tty，sudo 必须免密才跑得起来 |
| `SETENV` | launch 用 `sudo -E`；默认 `env_reset` 下没有该 tag 时 `-E` 会被拒绝 |
| 只授权 `/usr/bin/setpriv` | 形式上比 `NOPASSWD: ALL` 窄；需要按用户授权时把 `%sudo` 换成具体用户名 |

安装（文件名**不能含 `.`**，否则 sudo 会跳过）：

```bash
sudo cp ~/ros2_ws/src/ros2_botyard/dexhand_description/config/99_ethercat_cap /etc/sudoers.d/
sudo chown root:root /etc/sudoers.d/99_ethercat_cap
sudo chmod 0440 /etc/sudoers.d/99_ethercat_cap
sudo visudo -c       # 校验语法；改动建议用 sudo visudo -f 编辑，避免语法错误锁死 sudo
```

校验：

```bash
sudo -n -l | grep setpriv        # 应看到 (root) NOPASSWD: SETENV: /usr/bin/setpriv
```

> 若改用方案2（`cap_grant:=shell`）则本步可跳过。
> 注意：规则把 `setpriv` 的免密执行权授予整个 `%sudo` 组；由于 `setpriv` 能派生任意程序
> （如 `sudo setpriv --reuid=0 --regid=0 /bin/bash` 即可得 root shell），它形式虽比
> `NOPASSWD: ALL` 窄，实质权限仍然很高，请只在受控开发机上使用。

## 3. 启动方式

`communicate` / `port_name` 在所有 `hw_` 前缀 launch 里都不是显式声明的 launch 参数，
需要通过 `xacro_args` 传入：

```bash
# 不传网卡名，由驱动逐个非回环网卡自动探测
$ ros2 launch dexhand_bringup hw_dexhand_controller.launch.py \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT'

# 指定网卡（网卡名可用 ip -br link 查询）
$ ros2 launch dexhand_bringup hw_dexhand_controller.launch.py \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT port_name:=enp3s0'
```

## 4. 网卡选择（port_name 语义）

`communicate:=EtherCAT` 时 `port_name` 表示**网卡名**（不是 `/dev/xxx` 串口路径）。
判定与分支见
[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L310-L336)：

| port_name | 行为 | 日志 |
| --- | --- | --- |
| 本机存在的网卡名（如 `enp3s0`） | 固定在该网卡上连接 | `EtherCAT uses network interface <网卡>`（绿色高亮） |
| 空 | 由驱动逐个非回环网卡自动探测 | `port_name is empty, EtherCAT auto-detects the interface`，探测成功后打印 `automatically selected EtherCAT interface <网卡>`（驱动库输出） |
| 以 `/` 开头的设备路径（如 `/dev/dexhand`、`/dev/ttyACM0`） | 视为“未指定网卡”，按自动探测处理 | `port_name '...' is a device path, not a network interface; EtherCAT auto-detects the interface`（WARN） |
| 像网卡名但本机不存在（通常拼错） | **不做静默回退**，直接 configure 失败 | `port_name 'xxx' is not a network interface of this host; ... Refusing to fall back to auto-detect ...`（ERROR） |

设计意图（代码注释原文）：自动探测无法区分同型号的两只手，多只手必须逐只手显式指定网卡，
因此拒绝静默回退，避免两个 master 连到同一只手。

> 建议：首次连接后先用日志确认每只手落在哪块网卡上（`EtherCAT uses network interface ...`），
> 再进入调参/发指令阶段。

## 5. 从站识别依据

从站的筛选参数在 `EthercatOptions`
（`ethercat_types.hpp`，随 `dexhand_driver` 安装）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `expected_vendor` | `0x00000009` | 厂商 ID |
| `expected_product` | `0x00009252` | 产品 ID |
| `expected_revision` | `0x00000003` | 版本号 |
| `expected_serial` | `0` | `0` 表示**不筛选序列号** |

由于不校验序列号，自动探测按“网卡名（字母序）+ 厂商/产品/版本号”判定，
**同型号的多只手无法区分**：第二个实例可能扫到第一只手并连上，导致两个 master 控制同一只手
（先启动的掉线、后启动的连不上）。多只手必须逐只手显式指定各自的网卡，并建议用不同的
`namespace` 隔离 ROS 资源：

```bash
# 终端1：左手，网卡 enp3s0
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py namespace:=lh \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT port_name:=enp3s0'

# 终端2：右手，网卡 enp4s0
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py namespace:=rh \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT port_name:=enp4s0'
```

> 单进程两只手的 `hw_position_controller_two_dexhand.launch.py` 只透出 `lh_port_name`/`rh_port_name`
> （默认 RS485），未透出 `lh_communicate`/`rh_communicate`，因此该 launch 暂不支持单进程两只手走 EtherCAT。

## 6. 链路参数

`DexhandHardwareInterface` 用默认值构造 `EthercatOptions`，只把网卡名传进去
（[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L363-L368)），
其余参数目前**不可通过 ROS 参数配置**，如需调整要改驱动侧默认值：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `cycle_us` | `2000` | PDO 周期，即 **2 ms** |
| `command_ack_timeout_ms` | `100` | 命令确认超时 |
| `sdo_timeout_ms` | `700` | SDO 超时 |
| `max_recovery_attempts` | `11` | 单次恢复的总尝试次数 |

说明：`cycle_us` / `command_ack_timeout_ms` / `sdo_timeout_ms` / `max_recovery_attempts`
定义在 `ethercat_types.hpp`，
PDO/SDO 对象映射与 SOEM 状态封装在驱动库内部，属实现细节，上层仍只调用统一的 `execCmd*` 方法。

## 7. 数据面：话题 / 服务 / action

EtherCAT 与 RS485 共用同一套应用层接口，使用方式见主 README；下表只列出与 EtherCAT 相关的差异：

| 接口 | 名称 | 说明 |
| --- | --- | --- |
| 关节状态 | `/joint_states` | 由 `joint_state_broadcaster` 发布，所有后端一致 |
| 控制指令 | `/dexhand_controller/commands` | `std_msgs/Float64MultiArray`，三档模式（position_only / position+tactile / full_command），所有后端一致 |
| 指尖触觉 | `/dexhand_controller/state/tactiles` | fx/fy/fz，单位 0.01N，所有后端一致 |
| 网格触觉 | `/<hardware 名>_tactile/grid_tactiles` | **仅 EtherCAT 后端发布**，见下 |
| 手势 | `/dexhand_controller/do/gesture` | `dexhand_interfaces/action/DoGesture`，所有真机后端一致（必须真机，不能用 Mock） |

### 7.1 网格触觉（仅 EtherCAT）

网格触觉仅在 EtherCAT 后端启用：`start_io_()` 中 EtherCAT 连接成功后创建发布节点
（[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L598-L613)）。

- 话题 `~/grid_tactiles`；发布节点名 = 硬件组件名
  （[dexhand_single_macro.xacro](../dexhand_description/urdf/dexhand_single_macro.xacro#L33-L34)）加 `_tactile` 后缀
  （[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L604)），
  默认 `/botyard_dexhand_hardware_tactile/grid_tactiles`（带 `namespace` 时前缀加 `/<ns>`）。
- 类型 `dexhand_interfaces/msg/TactileGrid`，`SensorDataQoS`，100 Hz 定时器发布（executor 线程，不占 RT 控制循环）。
- 数据：5 指 × 116 点，单点 `fx/fy/fz` 为 `int8`，保持 EtherCAT wire 原始量纲、不做换算
  （`ethercat_types.hpp`）。
- 停止时先停发布再断开 EtherCAT，节点保留供下次 activate 复用。

```bash
$ ros2 topic list | grep grid_tactiles
$ ros2 topic echo /botyard_dexhand_hardware_tactile/grid_tactiles
```

## 8. 权限与启动方式

能力需求与一次性准备见第 2 节。下面说明 launch 提供的两种获取方式，
由 [ros2_control.launch.py](../dexhand_description/launch/ros2_control.launch.py)
的 `cap_grant` 参数选择，两个方案任选其一：

| 方案 | `cap_grant` 取值 | 用法 | 一次性准备 | 能力作用范围 |
| --- | --- | --- | --- | --- |
| 方案1：launch 内置 `sudo + setpriv` 借能力（默认） | `sudo` | 直接 `ros2 launch …` | 见第 2 节「前置准备」 | 仅 `ros2_control_node` 这一个进程 |
| 方案2：先在带能力的 shell 里，再启动 | `shell` | 见下方两步 | 每次开终端执行一次 `sudo setpriv` | 该 shell 及其全部子进程 |

两个方案的能力来源是同一套机制（`setpriv` 把 `cap_net_raw,cap_net_admin` 放进 ambient 集合）

- 方案1 由 launch 在启动节点的那一刻临时借一次，节点依然是当前用户身份；
- 方案2 在打开终端时借一次，此后这个 shell 里跑什么都自带能力，launch 不用再借
  （所以用 `cap_grant:=shell` 告诉它不要加前缀）。

注意：两个方案都**不要**把整条 launch 用 `sudo` 跑 —— 那样 `os.getuid()` 为 0，
节点会以 root 运行，与测试端分属不同用户，Fast DDS 共享内存不通。

`cap_grant` 也可以用环境变量指定：`DEXHAND_CAP_GRANT=shell ros2 launch …`；
环境变量对所有 launch 都生效（`hw_position_controller`、`hw_dexhand_joint_controller` 等未透传该参数的
launch 用这种方式切换）。`cap_grant` 只识别 `sudo`/`shell` 两个值，写成其它值按方案1 处理。

透出情况：`hw_dexhand_controller.launch.py` 显式透出 `cap_grant`
（[hw_dexhand_controller.launch.py](launch/hw_dexhand_controller.launch.py#L59-L64)），
其余 `hw_` launch 通过 [ros2_control.launch.py](../dexhand_description/launch/ros2_control.launch.py#L23-L29)
读取 `DEXHAND_CAP_GRANT` 环境变量（缺省 `sudo`）。

### 8.1 方案1：launch 内置 `sudo + setpriv` 借能力（默认）

启动命令就是最普通的这一条（前提是已完成第 2 节的免密配置）：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT'
```

`cap_grant:=sudo` 时，launch 给 `ros2_control_node` 加的前缀是：

```bash
sudo -E /usr/bin/setpriv \
  --reuid=$(id -u) --regid=$(id -g) --init-groups \
  --inh-caps=+net_raw,+net_admin --ambient-caps=+net_raw,+net_admin \
  /usr/bin/env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" <ros2_control_node> --ros-args …
```

四个环节各自解决一个问题：

| 环节 | 作用 |
| --- | --- |
| `sudo` | 取得“能给自己加能力”的资格（能力只有 root 或已有 `CAP_SETPCAP` 的进程才给得出） |
| `setpriv --reuid/--regid/--init-groups` | 把身份**立刻降回当前用户**（uid/gid/附加组），节点与 spawner、RViz 同用户，DDS 共享内存可用 |
| `--inh-caps/--ambient-caps` | 只借 `cap_net_raw,cap_net_admin` 这两个能力（放进 inheritable + ambient），其余能力一个不给 |
| `/usr/bin/env LD_LIBRARY_PATH=…` | `sudo` 会**强制清掉** `LD_LIBRARY_PATH`（`-E`、`--preserve-env` 都保不住），而节点要靠它找 ROS 库，所以在 `sudo` 之后显式注入 |

前提与限制：

- **当前用户可免密调用 `setpriv`**：launch 的子进程没有 tty，sudo 无法交互输密码，会直接失败或卡住。
  安装第 2 节「前置准备」里的 [99_ethercat_cap](../dexhand_description/config/99_ethercat_cap) 即可。
- **必须以普通用户启动整条 launch**，`sudo ros2 launch …` 会让降权失效（见上一条注意事项）。
- **该二进制上不能有 file capability**：否则每次执行都进 secure-exec（`AT_SECURE=1`），
  glibc 忽略 `LD_LIBRARY_PATH` → 报 `libbackward.so` 找不到（排查与清理见“已知坑”）。
- 优点：零系统配置、不用额外开终端，多只手 / 多终端各自直接启动即可。

自检（借到了能力、身份也没变）：

```bash
getpcaps $(pgrep -f ros2_control_node | head -n 1)        # 应含 cap_net_admin,cap_net_raw
ps -o user= -p $(pgrep -f ros2_control_node | head -n 1)  # 应为当前用户，不该是 root
```

### 8.2 方案2：在带能力的 shell 里启动（不依赖 sudo 免密）

先开一个“带能力”的 shell（**每次开终端做一次**）：

```bash
sudo -E /usr/bin/setpriv \
  --reuid=$(id -u) --regid=$(id -g) --init-groups \
  --inh-caps=+net_raw,+net_admin --ambient-caps=+net_raw,+net_admin \
  /usr/bin/env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" bash -l

getpcaps $$        # 应含 cap_net_admin,cap_net_raw，说明这个 shell 已经带能力
```

原理：与方案1 同一套 `setpriv`，只是从“每个节点启动时借一次”提前到“开 shell 时借一次”。
能力落在 shell 的 ambient 集合里，此后这个 shell 中每一次 `exec`（`ros2 launch` → `spawner` /
`ros2_control_node`）都自动带上，所以 launch 不必再加前缀：

```bash
ros2 launch dexhand_bringup hw_dexhand_controller.launch.py cap_grant:=shell \
  xacro_args:=' use_ros2_control:=true use_gazebo:=false use_mock_dexhand:=false communicate:=EtherCAT'
```

注意事项：

- 这里的 `sudo` 是**交互执行**的，可以正常输密码，因此本方案不要求 sudo 免密。
- 能力不跨终端：新开窗口要重新执行上面那条 `setpriv`；漏做的话节点没有能力，EtherCAT 会在 configure 阶段失败。
- `bash -l` 会重新读 profile，所以 `LD_LIBRARY_PATH` 必须用 `env` 显式带上，否则这个 shell 里的 ROS 命令可能找不到库。
- 作用范围是这个 shell 的**所有**子进程（比方案1 大）；命令跑完关掉这个 shell 即可回收。
- 在该 shell 里再执行 `sudo xxx`，得到的仍是普通 root 进程（能力不跨 sudo 边界），不影响使用。

自检（都在这个 shell 里执行）：

```bash
getpcaps $$                                               # shell 自身：应含 cap_net_admin,cap_net_raw
getpcaps $(pgrep -f ros2_control_node | head -n 1)        # 节点：同上
ps -o user= -p $(pgrep -f ros2_control_node | head -n 1)  # 应为当前用户，不该是 root
```

### 8.3 怎么选

| 场景 | 建议 |
| --- | --- |
| 单机使用，已完成第 2 节免密配置，平时直接 `ros2 launch` | 方案1（默认，零额外操作） |
| 不想给免密，或希望“只有这段时间才有能力” | 方案2 |
| 多只手 / 多个终端分别启动 | 两者都行；方案1 更省事，方案2 要给每个终端各开一次 |

## 9. 故障恢复机制（两层）

链路抖动/掉线时，driver 层与 hardware_interface 层各有一道防线：
driver 先在本地自愈，长时间自愈失败后由 hardware 层兜底重建 driver。

| 层 | 机制 | 参数 / 触发条件 |
| --- | --- | --- |
| driver | `recoverLink` 失败后**固定退避持续重试**，超过恢复窗口才 `FAULTED` 退出 PDO worker（避免 worker 线程永久退出导致 PDO 快照冻结） | 恢复窗口 3 s / 退避 500 ms（driver 侧常量，在预编译库内） |
| hardware | `io_thread` 中 `getDofsAndTactiles` **连续失败**累计达阈值 → 置 `fault_detected` 并退出 `io_thread` | 15 s，`kCommLostTimeout`（[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L41-L44)） |
| hardware | `io_thread` 轮询节流到 ~1 kHz，避免成功路径空转烧满 CPU、加剧调度抖动（EtherCAT 偶发丢帧的诱因之一） | 1 ms，`kIoPollPeriod`（[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L398-L401)） |
| hardware | `watchdog_thread_func()` **1 Hz 轮询**，两条故障感知路径：**A** `io_thread` 已因故障退出（`!io_thread_running && fault_detected`）；**B** `io_thread` 仍在运行但 `driver->isConnected()` 持续为 false（EtherCAT worker FAULTED）。两种情况都调用 `reconnect_driver_()` 重建 driver | A：立即重建；B：状态持续 6 s（`kDriverFaultConfirmTimeout`）确认后重建 |
| hardware | 重连时若上一轮手势被故障打断 | 预置 `result = FAILURE`，`io_thread` 从 `COMPLETED`（`awaiting_ack`）起步，等 controller 切回 TOPIC 且 `gesture_id` 归零再复位为 `IDLE` |

阈值关系：hardware 的 15 s **必须远大于** driver 的恢复窗口（3 s + 退避），
给 driver 留足自愈时间，只有 driver 确实救不回来时才兜底重建。

### 9.1 driver 层

`recoverLink` 在链路抖动后失败时，PDO worker 不再立即退出，而是以固定退避持续重试；
累计超过恢复窗口仍未恢复才置 `FAULTED` 并退出 worker。
该逻辑与常量都在预编译的 `libdexhand_driver.so` 内，仓库只提供头文件与 `.deb`。

### 9.2 hardware 层

`io_thread` 的失败计时与退出
（[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L494-L516)）：

- 首次 `getDofsAndTactiles` 失败即开始计时；期间以 1 s 限流打印错误，提示
  `Will trigger watchdog reconnect after 15s.`
- 恢复成功则清零计时。
- 连续失败达 15 s：若手势不在 `IDLE`/`COMPLETED`，标记“手势被故障打断”，
  置 `fault_detected = true`、`io_thread_running = false` 后退出线程，日志：
  `io_thread: communication fault sustained for 15s, exiting; watchdog will reconnect.`

`watchdog_thread_func()`（
[dexhand_hardware_interface.cpp](../dexhand_control/hardwares/dexhand_hardware_interface/src/dexhand_hardware_interface.cpp#L671-L743)）
1 Hz 轮询，有两条故障感知路径：

**路径 A：`io_thread` 已退出**（`!io_thread_running && fault_detected`）

1. join 已退出的 `io_thread`，清 `fault_detected`；
2. `reconnect_driver_()` 重建：`stop_io_()` → `dexhand_driver_.reset()` + `init_driver_()` → `start_io_()`；
3. 成功打印 `watchdog: driver reconnected, io_thread restarted.`；
   失败打印 `watchdog: reconnect failed, will retry next cycle.`，保持故障态下轮继续重试。

**路径 B：`io_thread` 仍在运行，但 `driver->isConnected()` 为 false**
（EtherCAT worker 已 FAULTED，而 `io_thread` 要等满 15 s 才退出，这里提前兜底）

- `isConnected()==false` 也可能只是 EtherCAT 的 RECOVERING 瞬态（driver 自愈窗口 3 s），
  因此要求状态**持续 6 s**（`kDriverFaultConfirmTimeout`）才确认故障，避免把能自愈的短抖动升级成一次完整重建。
- 确认故障后：先置 `io_thread_running = false` 并 join 确认退出，**任何时候都不允许对仍在运行的
  `io_thread` 直接 join**（否则 watchdog 会永久阻塞、丧失自愈能力）；join 后读取在途手势标志
  `gesture_in_flight_`，若手势被打断则置位 `gesture_interrupted_on_fault_`，再调用 `reconnect_driver_()` 重建。
- 成功打印 `watchdog: driver reconnected after sustained fault, io_thread restarted.`。

重建路径与首次 activate 复用同一套 `init_driver_()` / `start_io_()`，
保证重建后 IO 链路、网格触觉发布、手势终态语义与首次一致。停机顺序为先停 watchdog 再停 `io_thread`，
避免 watchdog 在退出期间触发重连。相关日志速查：

| 日志 | 含义 |
| --- | --- |
| `io_thread: getDofsAndTactiles failed for X.Xs (driver recovering or faulted); PDO worker may have exited...` | driver 正在自愈或已 FAULTED，尚未达到兜底阈值 |
| `io_thread: communication fault sustained for 15s, exiting; watchdog will reconnect.` | 已达阈值，io_thread 退出，等待 watchdog（路径 A） |
| `watchdog: io_thread exited due to communication fault, reconnecting driver...` | 路径 A：watchdog 开始重建 |
| `watchdog: driver reconnected, io_thread restarted.` | 路径 A 重建成功 |
| `watchdog: reconnect failed, will retry next cycle.` | 路径 A 重建失败，下轮重试 |
| `watchdog: driver reports not connected (EtherCAT recovering or FAULTED); allowing 6s self-healing window before forced rebuild.` | 路径 B：检测到 driver 未连接，进入 6 s 确认窗口 |
| `watchdog: driver not connected for 6s, forcing rebuild (stop io_thread, reconnect).` | 路径 B：确认故障，强制重建 |
| `watchdog: driver reconnected after sustained fault, io_thread restarted.` | 路径 B 重建成功 |
| `watchdog: reconnect after sustained fault failed, will retry next cycle.` | 路径 B 重建失败，下轮重试 |
| `==> start_io(): previous gesture was interrupted by a link fault; reporting FAILURE` | 上一轮手势被故障打断 |

## 10. 集成测试工具

[test_ethercat.cpp](test/test_ethercat.cpp) 是纯 ROS 客户端的 EtherCAT 链路集成测试
（不直接依赖 `dexhand_driver`，EtherCAT 连接由 ros2_control 侧负责）。**先启动带 `communicate:=EtherCAT`
的 launch，再运行本工具**：

```bash
$ ros2 run dexhand_bringup test_ethercat
$ ros2 run dexhand_bringup test_ethercat --namespace lh          # 带命名空间
$ ros2 run dexhand_bringup test_ethercat --controller dexhand_controller
$ ros2 run dexhand_bringup test_ethercat --grid-topic /botyard_dexhand_hardware_tactile/grid_tactiles
```

菜单项：

| 项 | 作用 | 涉及话题/接口 |
| --- | --- | --- |
| 1 `getDofs` | 读关节状态 | `/joint_states` |
| 2 `setDofs` | 写关节位置（position-only） | `~/commands` |
| 3 `getDofsAndTactiles` | 读关节 + 指尖触觉 | `/joint_states`，`~/state/tactiles` |
| 4 `setDofsAndTactiles` | 写关节位置 + 指尖力 | `~/commands` |
| 5 `doGesture` | 调手势 action | `~/do/gesture` |
| 6 `publish_grid_tactiles` | 订阅网格触觉并统计发布频率/非零点 | `~/grid_tactiles`（需 EtherCAT） |

工具会优先从控制器节点参数读取 `joints`/`tactiles` 顺序，读不到时退回内置默认顺序并给出排查提示。

## 11. 运行时问题

| 现象 | 处理 |
| --- | --- |
| 未收到 `/joint_states` 数据 | 确认 ros2_control 已启动、`joint_state_broadcaster` 为 active，且命名空间与 `--namespace` 一致 |
| 未收到 `.../state/tactiles` 数据 | 确认 `dexhand_controller` 为 active（`ros2 control list_controllers`） |
| 读不到 `/dexhand_controller` 参数 / 提示“使用内置默认顺序” | 依次排查：① 控制器是否加载；② `ros2_control_node` 属主是否为当前用户（为 `root` 说明用了 `sudo` 启动整条 launch）；③ 能力是否带上（`getpcaps`）；④ `--controller` 名字与命名空间是否匹配。顺序可能不一致时，**发送非零位置前务必核对** |
| 手势 action 不可用 | 确认 `dexhand_controller` 为 active，action 名默认 `/dexhand_controller/do/gesture` |
| 命令发了但手不动 | 检查硬件接口是否以 `communicate:=EtherCAT` 激活、EtherCAT 是否真的连上（看 ros2_control 日志） |
| 多只手时先启动的掉线、后启动的连不上 | 两个 EtherCAT master 连到了同一只手（自动探测无法区分同型号的手）：每个实例用 `port_name:=<各自网卡>` 显式指定，并确认日志为 `EtherCAT uses network interface <网卡>`；若出现 `port_name '...' is not a network interface of this host` 说明网卡名写错，改对后再启动，不要靠自动探测兜底 |
| 网格触觉一直是 0 条 | 网格触觉只在 EtherCAT 后端发布，用 `ros2 topic list` 核对话题名（多手带 prefix 时节点名会变） |
| `ros2_control_node` 不存在 / 启动即退出 | EtherCAT 连不上时它在启动阶段就崩，日志报 `Failed to 'configure' hardware`；`pgrep -f ros2_control_node` 为空即属此情况。检查灵巧手是否上电、是否连到本机网卡、网卡是否正确 |

## 12. 已知坑

| 问题 | 原因 | 解决方案 |
| --- | --- | --- |
| 建 EtherCAT socket / 置混杂模式失败 | 没有 `CAP_NET_RAW` / `CAP_NET_ADMIN` | 按第 2 节「前置准备」配置；方案2 则先在启动 launch 的那个 shell 里 `getpcaps $$` 确认确实带能力 |
| 方案1 下 sudo 卡在密码提示 | launch 子进程没有 tty，sudo 无法交互输入密码 | 安装仓库自带的 `dexhand_description/config/99_ethercat_cap`（见第 2 节「前置准备」）；或改用方案2 |
| `error while loading shared libraries: libbackward.so`（用 `setcap` 后） | 带 file capability 的进程进入 secure-exec（`AT_SECURE=1`），动态库加载器忽略 `LD_LIBRARY_PATH`；该二进制无 RUNPATH 且 `/opt/ros/jazzy/lib` 不在 `ld.so.conf` | **不要用 `setcap`**，改用上面两个方案（它们都不动二进制，不触发 secure-exec）。若历史操作已留下 file capability，先按下方命令清掉 |
| 同上报错（方案1 正常启动时也出现） | `sudo` 会**强制清掉 `LD_LIBRARY_PATH`**（`-E`、`--preserve-env` 都保不住），而 `ros2_control_node` 只能靠它找到 ROS 库 | 确认 launch 的 prefix 含 `/usr/bin/env LD_LIBRARY_PATH=…`（方案1），或方案2 的 `setpriv` shell 里带了该变量；用 `ldd /opt/ros/jazzy/lib/controller_manager/ros2_control_node \| grep "not found"` 定位缺失库 |
| 服务在 ROS 图上可见但读取超时 | 用 `sudo -E ros2 launch …` 时 `os.getuid()` 为 0，整条 launch 以 root 运行，与测试端分属不同用户，Fast DDS 共享内存不通 | 用普通用户启动整条 launch（两个方案都是）。无法避免时，测试端与它**同用户、同 ROS 域**（`ROS_HOME` 指到别处避免 `~/.ros` 锁文件），或两侧统一改用 `rmw_cyclonedds_cpp`（走 UDP，不用共享内存） |

### 清理历史残留的 file capability

`libbackward.so` 报错、且 `getcap` 显示二进制带 `cap_net_admin,cap_net_raw` 时照做：

```bash
# 1) 排查：列出 /opt/ros 下所有带 file capability 的文件（正常应无输出）
sudo getcap -r /opt/ros/jazzy

# 2) 清理（去掉 file cap，能力改由方案1/方案2 提供）
sudo setcap -r /opt/ros/jazzy/lib/controller_manager/ros2_control_node

# 3) 确认（应无输出）与验证
getcap -v /opt/ros/jazzy/lib/controller_manager/ros2_control_node
sudo -E /usr/bin/setpriv --reuid=$(id -u) --regid=$(id -g) --init-groups \
  --inh-caps=+net_raw,+net_admin --ambient-caps=+net_raw,+net_admin \
  /usr/bin/env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
  timeout 6 /opt/ros/jazzy/lib/controller_manager/ros2_control_node \
  --ros-args --params-file ~/ros2_ws/install/dexhand_description/share/dexhand_description/config/ros2_controllers.yaml
# 期望看到 controller_manager 正常启动，而不是 libbackward.so 加载失败
```

注意：`setcap -r` 是把这颗能力**移除**（能力改由方案1/方案2 提供），不要改成重新 `setcap` 打上去；
ROS 升级替换该二进制后 file cap 会丢，届时按上面的第 1 步重新确认一次即可。

## 13. 安全提示

确认灵巧手周围无障碍物后再发送非零目标位置或执行手势，首次建议小幅度、低速验证。

## 14. 手势调用与取消

先按第 3 节启动控制节点，等日志出现 `Configured and activated dexhand_controller`。

```bash
# 发送手势；gesture_id：1五指握 2四指握 3三指握 4四指扣握 5五指扣握 6二指捏合 7三指捏合 8爪式 9夹式 10钳式 255张开
ros2 action send_goal /dexhand_controller/do/gesture \
  dexhand_interfaces/action/DoGesture "{gesture_id: 1}" --feedback

# 取消（ROS 2 Action 自带的取消服务）；也可在 send_goal 终端按 Ctrl+C
ros2 service call /dexhand_controller/do/gesture/_action/cancel_goal \
  action_msgs/srv/CancelGoal "{}"
```

- goal 被接受后反馈 `state: 4`（BUSY，含执行与收尾）；一次只允许一个手势，已有动作时新 goal 被拒绝。
- 取消：终端返回 `return_code=0` 且 `goals_canceling` 非空，表示请求已接受，最终返回 `CANCELED` / `result: 6`；`goals_canceling=[]` 说明取消发得太早或动作已结束。取消不代表已物理停止，通信故障时尤甚，实际效果以真机观察为准。
- 正常完成返回 `SUCCEEDED` / `result: 1`；硬件超时返回 `ABORTED` / `result: 3`。

### 14.1 连续压力测试（可选）

[test/gesture_soak_test.sh](test/gesture_soak_test.sh) 自动交替发送手势（默认 `gesture_id=8` 与 `255`，间隔 5 s），一旦返回非 `SUCCESS(1)` 即停止并记录日志，同时向 `/dexhand_controller/commands` 发送恢复位姿（J2 伸长、其余归零）尝试复位。

```bash
cd dexhand_bringup/test && bash gesture_soak_test.sh
```

可用环境变量覆盖：`PRIMARY_GESTURE_ID`、`SECONDARY_GESTURE_ID`、`ACTION_INTERVAL_SEC`、`CMD_TIMEOUT_SEC`、`LOG_FILE`、`RECOVERY_ENABLED`（`0` 禁用恢复）、`COMMAND_TOPIC`。恢复位姿的关节顺序需与 `dexhand_bringup/config/dexhand_controller.yaml` 的 `joints` 一致。
