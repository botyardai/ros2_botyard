#!/usr/bin/env bash
#
# dexhand 手势连续压力测试脚本
#
# 通过 ROS 2 指令(ros2 action send_goal)驱动 /dexhand_controller/do/gesture:
#   1. 发送一次 gesture_id=8 的指令;
#   2. 间隔 5 秒后再发送一次 gesture_id=255 的指令, 之后每 5 秒发一次动作;
#   3. 以上循环持续执行, 直到任意一次返回的 result 不是 SUCCESS(1);
#   4. 一旦出现非 SUCCESS, 立即停止后续发送, 记录并把当前报错与状态打印到终端和日志文件。
#
# 用法:
#   ros2 launch dexhand_bringup hw_dexhand_controller.launch.py ...   # 终端 1 启动控制器
#   bash gesture_soak_test.sh                                        # 终端 2 运行本脚本
#
# 可通过环境变量覆盖默认配置(单位为秒):
#   PRIMARY_GESTURE_ID=8 SECONDARY_GESTURE_ID=255 \
#   ACTION_INTERVAL_SEC=5 CMD_TIMEOUT_SEC=60 \
#   ACTION_NAME=/dexhand_controller/do/gesture LOG_FILE=/tmp/gesture_soak.log \
#   bash gesture_soak_test.sh

set -u

ACTION_NAME="${ACTION_NAME:-/dexhand_controller/do/gesture}"
ACTION_TYPE="${ACTION_TYPE:-dexhand_interfaces/action/DoGesture}"
PRIMARY_GESTURE_ID="${PRIMARY_GESTURE_ID:-8}"
SECONDARY_GESTURE_ID="${SECONDARY_GESTURE_ID:-255}"
ACTION_INTERVAL_SEC="${ACTION_INTERVAL_SEC:-5}"      # 相邻两次动作的间隔: 5 秒
CMD_TIMEOUT_SEC="${CMD_TIMEOUT_SEC:-60}"             # 单次 action 等待上限
LOG_FILE="${LOG_FILE:-$(pwd)/gesture_soak_$(date +%Y%m%d_%H%M%S).log}"
# 故障恢复：向控制器 commands topic 发送 J2 伸长、其余关节归零的恢复位姿
COMMAND_TOPIC="${COMMAND_TOPIC:-/dexhand_controller/commands}"
RECOVERY_ENABLED="${RECOVERY_ENABLED:-1}"            # 1=启用故障恢复, 0=禁用

# GestureResult 编码, 见 dexhand_interfaces/include/dexhand_interfaces/gesture_enums.hpp
SUCCESS_RESULT=1

# 若未配置 ROS 2 环境, 尝试自动 source 工作空间的安装环境
if ! command -v ros2 >/dev/null 2>&1; then
  if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$HOME/ros2_ws/install/setup.bash"
  fi
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误: 找不到 ros2 命令, 请先 source ROS 2 环境 (source /opt/ros/<distro>/setup.bash)" >&2
  exit 2
fi

result_name() {
  case "$1" in
    0) echo "IDLE" ;;
    1) echo "SUCCESS" ;;
    2) echo "FAILURE" ;;
    3) echo "TIMEOUT" ;;
    4) echo "BUSY" ;;
    5) echo "INVALID" ;;
    6) echo "CANCELLED" ;;
    *) echo "UNKNOWN" ;;
  esac
}

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$LOG_FILE"
}

# 发送一次手势指令并回显结果。
# 成功时返回 0 并输出 result 值; 失败(命令异常/超时/非 SUCCESS)时返回 1。
send_gesture() {
  local gesture_id="$1"
  local output rc result description

  log ">>> 发送手势: gesture_id=${gesture_id} (action=${ACTION_NAME})"
  output="$(timeout "${CMD_TIMEOUT_SEC}s" ros2 action send_goal \
    "$ACTION_NAME" "$ACTION_TYPE" "{gesture_id: ${gesture_id}}" 2>&1)"
  rc=$?

  if [ "$rc" -eq 124 ]; then
    log "!!! 发送手势 ${gesture_id} 超时: ${CMD_TIMEOUT_SEC}s 内未收到最终结果"
    log "--- 原始输出 ---"
    log "$output"
    return 1
  fi
  if [ "$rc" -ne 0 ]; then
    log "!!! 发送手势 ${gesture_id} 失败: ros2 命令退出码 ${rc}"
    log "--- 原始输出 ---"
    log "$output"
    return 1
  fi

  result="$(printf '%s\n' "$output" | sed -n 's/.*result:[[:space:]]*\([0-9]\{1,\}\).*/\1/p' | tail -n 1)"
  if [ -z "$result" ]; then
    log "!!! 发送手势 ${gesture_id} 未能解析到 result 字段"
    log "--- 原始输出 ---"
    log "$output"
    return 1
  fi

  description="$(printf '%s\n' "$output" | sed -n 's/.*description:[[:space:]]*//p' | tail -n 1)"

  if [ "$result" -ne "$SUCCESS_RESULT" ]; then
    log "!!! 手势 ${gesture_id} 返回非 SUCCESS: result=${result} ($(result_name "$result"))"
    if [ -n "$description" ]; then
      log "!!! 报错描述: ${description}"
    fi
    log "--- 失败时状态 ---"
    log "gesture_id=${gesture_id} result=${result} ($(result_name "$result")) description=${description:-<空>}"
    log "--- 原始输出 ---"
    log "$output"
    return 1
  fi

  log "<<< 手势 ${gesture_id} 完成: result=${result} ($(result_name "$result"))"
  return 0
}

# 故障恢复：发送 J2 关节伸长(90°=1.5708rad)、其余关节归零的 topic 指令。
# 关节顺序与 dexhand_controller.yaml 的 joints 参数一致：
#   FAJ3 FAJ1 THJ4 THJ3 THJ2 THJ1 FFJ4 FFJ3 FFJ2 MFJ4 MFJ3 MFJ2 RFJ4 RFJ3 RFJ2 LFJ4 LFJ3 LFJ2
# J2 关节(THJ2 FFJ2 MFJ2 RFJ2 LFJ2)位于下标 4 8 11 14 17，设为 π/2；其余为 0。
send_recovery_pose() {
  if [ "${RECOVERY_ENABLED}" != "1" ]; then
    log "恢复位姿已禁用 (RECOVERY_ENABLED=0)，跳过发送"
    return 0
  fi
  log ">>> 发送恢复位姿: J2 伸长, 其余归零 (topic=${COMMAND_TOPIC})"
  # 等待控制器回到 TOPIC/IDLE 相位（cleanup 宽限 1s + 余量）
  sleep 2
  local recovery_rc
  timeout 10s ros2 topic pub --once "${COMMAND_TOPIC}" std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 1.5708, 0.0, 0.0, 1.5708, 0.0, 0.0, 1.5708]}" 2>&1 | tee -a "$LOG_FILE"
  recovery_rc=$?
  if [ "$recovery_rc" -eq 0 ]; then
    log "<<< 恢复位姿已发送"
  else
    log "!!! 恢复位姿发送失败 (rc=${recovery_rc})"
  fi
  return "$recovery_rc"
}

ROUND=0
log "手势压力测试开始: 主手势=${PRIMARY_GESTURE_ID}, 次手势=${SECONDARY_GESTURE_ID}, 动作间隔=${ACTION_INTERVAL_SEC}s"
log "action=${ACTION_NAME}, 单次超时=${CMD_TIMEOUT_SEC}s, 日志=${LOG_FILE}"

while true; do
  ROUND=$((ROUND + 1))
  log "===== 第 ${ROUND} 轮 ====="

  if ! send_gesture "$PRIMARY_GESTURE_ID"; then
    log "===== 检测到异常, 停止后续发送 (第 ${ROUND} 轮 主手势 ${PRIMARY_GESTURE_ID}) ====="
    send_recovery_pose
    exit 1
  fi

  log "等待 ${ACTION_INTERVAL_SEC}s 后发送次手势 ${SECONDARY_GESTURE_ID}"
  sleep "$ACTION_INTERVAL_SEC"

  if ! send_gesture "$SECONDARY_GESTURE_ID"; then
    log "===== 检测到异常, 停止后续发送 (第 ${ROUND} 轮 次手势 ${SECONDARY_GESTURE_ID}) ====="
    send_recovery_pose
    exit 1
  fi

  log "等待 ${ACTION_INTERVAL_SEC}s 进入下一轮"
  sleep "$ACTION_INTERVAL_SEC"
done
