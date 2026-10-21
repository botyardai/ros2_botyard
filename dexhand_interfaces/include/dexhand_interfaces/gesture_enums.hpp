#pragma once

#include <cstdint>
#include <string_view>

namespace dexhand_interfaces
{

enum class DexhandMode : std::uint8_t
{
  TOPIC = 1,
  GESTURE = 2,
};

enum class GestureResult : std::uint8_t
{
  IDLE = 0,
  SUCCESS = 1,
  FAILURE = 2,
  TIMEOUT = 3,
  BUSY = 4,
  INVALID = 5,
  CANCELLED = 6,
};

// action 增加一个取消路径
constexpr bool is_terminal_result(const GestureResult result)
{
  return result == GestureResult::SUCCESS || result == GestureResult::FAILURE ||
         result == GestureResult::TIMEOUT || result == GestureResult::INVALID ||
         result == GestureResult::CANCELLED;
}

constexpr std::uint8_t to_uint8(const DexhandMode mode)
{
  return static_cast<std::uint8_t>(mode);
}

constexpr std::uint8_t to_uint8(const GestureResult result)
{
  return static_cast<std::uint8_t>(result);
}

constexpr std::string_view to_string(const GestureResult result)
{
  switch (result) {
    case GestureResult::IDLE: return "IDLE";
    case GestureResult::SUCCESS: return "SUCCESS";
    case GestureResult::FAILURE: return "FAILURE";
    case GestureResult::TIMEOUT: return "TIMEOUT";
    case GestureResult::BUSY: return "BUSY";
    case GestureResult::INVALID: return "INVALID";
    case GestureResult::CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

}  // namespace dexhand_interfaces
