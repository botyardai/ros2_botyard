// Copyright 2026 botayrd.ai Robotics
//

#ifndef DEXHAND_CONTROLLER__GESTURE_SESSION_HPP_
#define DEXHAND_CONTROLLER__GESTURE_SESSION_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>

#include "dexhand_interfaces/gesture_enums.hpp"

namespace dexhand_controller
{

using TimePoint = std::chrono::steady_clock::time_point;
// 与 rclcpp_action::GoalUUID 同布局，adapter 可直接传入，本模块不依赖 ROS。
using GoalUuid = std::array<std::uint8_t, 16>;

struct GestureSessionConfig
{
  std::chrono::milliseconds result_timeout{10000};  // 端到端兜底 = gesture_result_timeout_ms
  std::chrono::milliseconds cleanup_grace{1000};    // 收尾：等 hardware 回报 IDLE 的宽限期
  std::chrono::milliseconds feedback_period{100};   // BUSY feedback 节流周期
};

// 跨 RT seam 的唯一“一致快照”：控制权标记与 mode/gesture_id 合并成一个值，
// update() 侧无需再把两个独立字段拼成一份命令。
// 不变量：claimed==false ⇒ mode==TOPIC 且 gesture_id==0。
struct ChannelCommand
{
  bool claimed = false;
  dexhand_interfaces::DexhandMode mode = dexhand_interfaces::DexhandMode::TOPIC;
  std::uint8_t gesture_id = 0;
};

// RT -> 非 RT 的观测。valid==false 表示读不到/非法（255），绝不等于 IDLE。
struct HardwareSample
{
  dexhand_interfaces::GestureResult result = dexhand_interfaces::GestureResult::IDLE;
  bool valid = false;
};

// action 结果输出端口：adapter 持有 goal handle，因此本模块保持与 ROS 解耦。
class GoalObserver
{
public:
  virtual ~GoalObserver() = default;

  virtual bool is_canceling() const = 0;
  virtual void succeeded(dexhand_interfaces::GestureResult result) = 0;
  virtual void canceled(dexhand_interfaces::GestureResult result) = 0;  // 仅 canceling 时调用
  virtual void aborted(dexhand_interfaces::GestureResult result) = 0;
  virtual void busy_feedback() = 0;
};

// 跨 seam 输出 port：session 写、RT 控制环读。production 用无锁 mailbox 实现。
class ChannelPort
{
public:
  virtual ~ChannelPort() = default;

  virtual void publish_command(const ChannelCommand & command) = 0;
};

// 手势通道的“租约持有者 + 生命周期状态机”。
// 线程约定：本类所有方法只在同一个非 RT 线程（同一个 mutually-exclusive callback group，
// 调用方持锁）调用；RT 控制环只通过 ChannelPort 与观测原子交换数据，绝不调用本类。
class GestureSession
{
public:
  using WarnFn = std::function<void(std::string_view)>;

  GestureSession(GestureSessionConfig config, ChannelPort & port, WarnFn warn = {});

  // 生命周期：controller 的 on_activate / on_deactivate
  void activate(TimePoint now);
  void deactivate(TimePoint now);

  // action 回调入口
  bool reserve(const GoalUuid & uuid, std::uint8_t gesture_id);             // handle_goal
  bool start(const GoalUuid & uuid, std::shared_ptr<GoalObserver> observer, TimePoint now);  // handle_accepted
  bool accept_cancel(const GoalUuid & uuid);                                // handle_cancel

  // 10ms timer 驱动：终态判定、端到端超时、收尾握手、feedback 全部在这里。
  void tick(TimePoint now, const HardwareSample & sample);

  ChannelCommand channel_command() const;  // 停用时直接落 command interface 用
  bool active() const;
  bool claimed() const;  // topic 订阅回调据此丢弃手势期间的命令

private:
  // 内部相位模型（仅实现内部使用，不暴露为 caller 的分支逻辑）
  enum class Phase { Inactive, Cleanup, Idle, Reserved, Executing };

  void finish(TimePoint now, dexhand_interfaces::GestureResult result);
  void release_to_cleanup(TimePoint now);
  void publish();

  GestureSessionConfig config_;
  ChannelPort & port_;
  WarnFn warn_;

  Phase phase_{Phase::Inactive};
  GoalUuid reserved_uuid_{};
  std::uint8_t reserved_gesture_id_{0};
  GoalUuid active_uuid_{};
  std::uint8_t active_gesture_id_{0};
  std::shared_ptr<GoalObserver> observer_;
  bool cancel_dispatched_{false};
  HardwareSample last_sample_{};
  TimePoint result_deadline_{};
  TimePoint cleanup_deadline_{};
  TimePoint next_feedback_{};
};

}  // namespace dexhand_controller

#endif  // DEXHAND_CONTROLLER__GESTURE_SESSION_HPP_
