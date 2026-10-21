// Copyright 2026 botayrd.ai Robotics
//

#include "dexhand_controller/gesture_session.hpp"

#include <utility>

namespace dexhand_controller
{
using dexhand_interfaces::DexhandMode;
using dexhand_interfaces::GestureResult;

namespace
{
bool is_terminal(const GestureResult result)
{
  return dexhand_interfaces::is_terminal_result(result);
}
}  // namespace

GestureSession::GestureSession(GestureSessionConfig config, ChannelPort & port, WarnFn warn)
: config_(config), port_(port), warn_(std::move(warn)) {}

void GestureSession::activate(const TimePoint now)
{
  // 激活后先走一轮 TOPIC/IDLE 握手，确认 hardware 处于空闲再允许接单（与停用时同路径）。
  reserved_uuid_ = GoalUuid{};
  reserved_gesture_id_ = 0;
  active_uuid_ = GoalUuid{};
  active_gesture_id_ = 0;
  observer_.reset();
  cancel_dispatched_ = false;
  last_sample_ = HardwareSample{};
  next_feedback_ = now;
  result_deadline_ = now;
  phase_ = Phase::Cleanup;
  cleanup_deadline_ = now + config_.cleanup_grace;
  publish();
}

void GestureSession::deactivate(const TimePoint /*now*/)
{
  if (observer_) {
    observer_->aborted(GestureResult::FAILURE);
    observer_.reset();
  }
  reserved_uuid_ = GoalUuid{};
  reserved_gesture_id_ = 0;
  active_uuid_ = GoalUuid{};
  active_gesture_id_ = 0;
  cancel_dispatched_ = false;
  last_sample_ = HardwareSample{};
  phase_ = Phase::Inactive;
  publish();
}

bool GestureSession::reserve(const GoalUuid & uuid, const std::uint8_t gesture_id)
{
  // 只有空闲、且 ID 合法（0 为非法）时才接单；接单即占用通道。
  if (phase_ != Phase::Idle || gesture_id == 0) {
    return false;
  }
  phase_ = Phase::Reserved;
  reserved_uuid_ = uuid;
  reserved_gesture_id_ = gesture_id;
  publish();
  return true;
}

bool GestureSession::start(
  const GoalUuid & uuid, std::shared_ptr<GoalObserver> observer, const TimePoint now)
{
  if (phase_ != Phase::Reserved || uuid != reserved_uuid_) {
    return false;
  }
  if (!observer) {
    // 预约匹配但未提供 GoalObserver：释放控制权，避免通道占位永久停留在 GESTURE。
    release_to_cleanup(now);
    return false;
  }
  active_uuid_ = uuid;
  active_gesture_id_ = reserved_gesture_id_;
  reserved_uuid_ = GoalUuid{};
  reserved_gesture_id_ = 0;
  observer_ = std::move(observer);
  cancel_dispatched_ = false;
  result_deadline_ = now + config_.result_timeout;
  next_feedback_ = now;
  phase_ = Phase::Executing;
  publish();
  return true;
}

bool GestureSession::accept_cancel(const GoalUuid & uuid)
{
  if (phase_ != Phase::Executing || uuid != active_uuid_) {
    return false;
  }
  // 已经读到终态就不再接受取消。rclcpp 在本回调返回后才置 CANCELING，
  // 因此停止命令由下一次 tick 依据 observer_->is_canceling() 下发。
  if (last_sample_.valid && is_terminal(last_sample_.result)) {
    return false;
  }
  return true;
}

void GestureSession::tick(const TimePoint now, const HardwareSample & sample)
{
  if (sample.valid) {
    last_sample_ = sample;
  }

  switch (phase_) {
    case Phase::Inactive:
    case Phase::Idle:
    case Phase::Reserved:
      // 预约中的 goal 不受理任何硬件推进（等 handle_accepted）。
      return;

    case Phase::Cleanup:
      // 等 hardware 确认 IDLE 才放行新 goal；超宽限强放，避免后续 goal 被永久拒绝。
      if (sample.valid && sample.result == GestureResult::IDLE) {
        phase_ = Phase::Idle;
      } else if (now >= cleanup_deadline_) {
        if (warn_) {
          warn_("Hardware did not confirm IDLE within the cleanup grace period; releasing lease");
        }
        phase_ = Phase::Idle;
      }
      return;

    case Phase::Executing: {
      if (observer_ && observer_->is_canceling() && !cancel_dispatched_) {
        // 停止命令只下发一次，此后保持 TOPIC+id 直到 hardware 回报终态。
        cancel_dispatched_ = true;
        publish();
      }
      if (sample.valid && is_terminal(sample.result)) {
        finish(now, sample.result);
        return;
      }
      if (now >= result_deadline_) {
        if (warn_) {
          warn_("Gesture produced no terminal result before the deadline; finishing with TIMEOUT");
        }
        finish(now, GestureResult::TIMEOUT);
        return;
      }
      if (now >= next_feedback_) {
        if (observer_) {
          observer_->busy_feedback();
        }
        next_feedback_ = now + config_.feedback_period;
      }
      return;
    }
  }
}

ChannelCommand GestureSession::channel_command() const
{
  if (!claimed()) {
    return ChannelCommand{};
  }
  if (phase_ == Phase::Reserved) {
    // 已占用通道但尚未启动：下发 TOPIC+0，暂停 topic 转发且不触发 hardware 手势。
    return ChannelCommand{true, DexhandMode::TOPIC, 0};
  }
  // 取消在途时切 TOPIC+id 请求停止；正常执行时 GESTURE+id（id 恒非 0）。
  const auto mode = cancel_dispatched_ ? DexhandMode::TOPIC : DexhandMode::GESTURE;
  return ChannelCommand{true, mode, active_gesture_id_};
}

bool GestureSession::active() const
{
  return phase_ != Phase::Inactive;
}

bool GestureSession::claimed() const
{
  return phase_ == Phase::Reserved || phase_ == Phase::Executing;
}

void GestureSession::finish(const TimePoint now, const GestureResult result_in)
{
  const auto observer = observer_;
  observer_.reset();
  active_uuid_ = GoalUuid{};
  active_gesture_id_ = 0;
  cancel_dispatched_ = false;

  if (observer) {
    auto result = result_in;
    const bool canceling = observer->is_canceling();
    // 取消已接受但底层恰好正常结束：任务已不再执行，按接受的取消请求收尾。
    if (canceling && result == GestureResult::SUCCESS) {
      result = GestureResult::CANCELLED;
    }
    // 只有已进入 CANCELING 的 goal 才能调用 canceled()。
    if (result == GestureResult::CANCELLED && canceling) {
      observer->canceled(result);
    } else if (result == GestureResult::SUCCESS) {
      observer->succeeded(result);
    } else {
      observer->aborted(result);
    }
  }

  release_to_cleanup(now);
}

void GestureSession::release_to_cleanup(const TimePoint now)
{
  phase_ = Phase::Cleanup;
  cleanup_deadline_ = now + config_.cleanup_grace;
  publish();
}

void GestureSession::publish()
{
  port_.publish_command(channel_command());
}

}  // namespace dexhand_controller
