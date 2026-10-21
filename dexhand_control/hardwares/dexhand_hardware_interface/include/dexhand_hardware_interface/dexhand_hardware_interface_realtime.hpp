#pragma once

#include <thread>
#include <atomic>
#include <realtime_tools/realtime_buffer.hpp>
#include "dexhand_driver/dexhand_driver.hpp"

namespace dexhand_hardware_interface {

using DataWrapper = dexhand_driver::DataWrapper;

struct DexhandRealtimeBuffers {
  realtime_tools::RealtimeBuffer<DataWrapper> state_buffer;
  realtime_tools::RealtimeBuffer<DataWrapper> command_buffer;
  std::atomic<bool> io_thread_running{false};
  std::thread io_thread;
};

} // namespace dexhand_hardware_interface
