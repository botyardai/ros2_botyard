#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include <realtime_tools/realtime_buffer.hpp>

#include "dexhand_driver/dexhand_driver.hpp"
#include "dexhand_interfaces/gesture_enums.hpp"

namespace dexhand_hardware_interface
{

using DataWrapper = dexhand_driver::DataWrapper;

struct DexhandModeCommand
{
  // mode、gesture_id 和 topic 数据必须作为同一份快照提交，避免跨周期拼接。
  dexhand_interfaces::DexhandMode mode = dexhand_interfaces::DexhandMode::TOPIC;
  std::uint8_t gesture_id = 0;
  std::uint64_t synchronized_gesture_sequence = 0;
  DataWrapper data;
};

struct GestureFeedback
{
  dexhand_interfaces::GestureResult result = dexhand_interfaces::GestureResult::IDLE;
  std::uint64_t command_sequence = 0;
  DataWrapper completed_command;
};

struct DexhandRealtimeBuffers
{
  realtime_tools::RealtimeBuffer<DataWrapper> state_buffer;
  realtime_tools::RealtimeBuffer<DexhandModeCommand> command_buffer;
  realtime_tools::RealtimeBuffer<GestureFeedback> gesture_feedback_buffer;
  std::atomic<bool> io_thread_running{false};
  std::thread io_thread;
};

}  // namespace dexhand_hardware_interface
