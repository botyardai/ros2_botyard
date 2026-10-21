#include <cstdint>

#include "gtest/gtest.h"

#include "dexhand_interfaces/gesture_enums.hpp"

namespace
{

using dexhand_interfaces::DexhandMode;
using dexhand_interfaces::GestureResult;

TEST(GestureEnums, ProtocolValuesRemainStable)
{
  EXPECT_EQ(dexhand_interfaces::to_uint8(DexhandMode::TOPIC), 1U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(DexhandMode::GESTURE), 2U);

  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::IDLE), 0U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::SUCCESS), 1U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::FAILURE), 2U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::TIMEOUT), 3U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::BUSY), 4U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::INVALID), 5U);
  EXPECT_EQ(dexhand_interfaces::to_uint8(GestureResult::CANCELLED), 6U);
}

TEST(GestureEnums, ActionResultsAreTerminal)
{
  EXPECT_FALSE(dexhand_interfaces::is_terminal_result(GestureResult::IDLE));
  EXPECT_TRUE(dexhand_interfaces::is_terminal_result(GestureResult::SUCCESS));
  EXPECT_TRUE(dexhand_interfaces::is_terminal_result(GestureResult::FAILURE));
  EXPECT_TRUE(dexhand_interfaces::is_terminal_result(GestureResult::TIMEOUT));
  EXPECT_FALSE(dexhand_interfaces::is_terminal_result(GestureResult::BUSY));
  EXPECT_TRUE(dexhand_interfaces::is_terminal_result(GestureResult::INVALID));
  EXPECT_TRUE(dexhand_interfaces::is_terminal_result(GestureResult::CANCELLED));
}

}  // namespace
