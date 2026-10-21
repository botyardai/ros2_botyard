// Copyright 2026 botyard.ai Robotics

#include <chrono>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "dexhand_interfaces/gesture_enums.hpp"
#include "dexhand_interfaces/action/do_gesture.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using dexhand_interfaces::GestureResult;
using DoGesture = dexhand_interfaces::action::DoGesture;

struct Arguments
{
  std::string action = "/dexhand_controller/do/gesture";
  std::chrono::milliseconds server_wait{10000};
  std::chrono::milliseconds response_timeout{15000};
  std::optional<std::uint8_t> gesture_id;
  std::optional<GestureResult> expected_result;
  bool show_help = false;
};

std::string trim(const std::string & value)
{
  std::size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

std::uint32_t parse_uint32(const std::string & text, const char * name)
{
  std::size_t parsed = 0;
  const auto value = std::stoull(text, &parsed, 0);
  if (parsed != text.size() || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::out_of_range(std::string(name) + " is outside the uint32 range");
  }
  return static_cast<std::uint32_t>(value);
}

GestureResult parse_result(const std::string & text)
{
  for (const auto result : {
      GestureResult::IDLE, GestureResult::SUCCESS, GestureResult::FAILURE,
      GestureResult::TIMEOUT, GestureResult::BUSY, GestureResult::INVALID,
      GestureResult::CANCELLED
    })
  {
    if (dexhand_interfaces::to_string(result) == text) {
      return result;
    }
  }
  throw std::invalid_argument("unknown result: " + text);
}

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options] [--ros-args ...]\n"
    << "  --action NAME                  action name (default: "
    << "/dexhand_controller/do/gesture)\n"
    << "  --gesture-id ID               execute once instead of reading stdin\n"
    << "  --expect RESULT               expected result for one-shot mode\n"
    << "  --server-wait-ms MS           action discovery timeout (default: 10000)\n"
    << "  --response-timeout-ms MS      action result timeout (default: 15000)\n"
    << "  -h, --help                    show this help\n"
    << "RESULT: IDLE, SUCCESS, FAILURE, TIMEOUT, BUSY, INVALID, CANCELLED\n";
}

std::string take_value(
  const std::vector<std::string> & arguments, std::size_t & index,
  const std::string & option)
{
  if (++index >= arguments.size()) {
    throw std::invalid_argument(option + " requires a value");
  }
  return arguments[index];
}

bool parse_arguments(const std::vector<std::string> & arguments, Arguments & args)
{
  try {
    for (std::size_t index = 1; index < arguments.size(); ++index) {
      const auto & option = arguments[index];
      if (option == "-h" || option == "--help") {
        args.show_help = true;
        return true;
      }
      if (option == "--action") {
        args.action = take_value(arguments, index, option);
      } else if (option == "--gesture-id") {
        const auto value = parse_uint32(take_value(arguments, index, option), "gesture-id");
        if (value == 0 || value > 255) {
          throw std::out_of_range("gesture-id must be in [1, 255]");
        }
        args.gesture_id = static_cast<std::uint8_t>(value);
      } else if (option == "--expect") {
        args.expected_result = parse_result(take_value(arguments, index, option));
      } else if (option == "--server-wait-ms") {
        const auto value = parse_uint32(take_value(arguments, index, option), "server-wait-ms");
        if (value == 0) {
          throw std::out_of_range("server-wait-ms must be greater than zero");
        }
        args.server_wait = std::chrono::milliseconds(value);
      } else if (option == "--response-timeout-ms") {
        const auto value = parse_uint32(
          take_value(arguments, index, option), "response-timeout-ms");
        if (value == 0) {
          throw std::out_of_range("response-timeout-ms must be greater than zero");
        }
        args.response_timeout = std::chrono::milliseconds(value);
      } else {
        throw std::invalid_argument("unknown argument: " + option);
      }
    }
    if (args.action.empty()) {
      throw std::invalid_argument("action name must not be empty");
    }
    if (args.expected_result && !args.gesture_id) {
      throw std::invalid_argument("--expect requires --gesture-id");
    }
  } catch (const std::exception & error) {
    std::cerr << "Argument error: " << error.what() << '\n';
    print_usage(arguments.front().c_str());
    return false;
  }
  return true;
}

std::optional<GestureResult> call_gesture(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<DoGesture>::SharedPtr & client,
  const std::uint8_t gesture_id,
  const std::chrono::milliseconds timeout)
{
  DoGesture::Goal request;
  request.gesture_id = gesture_id;
  auto goal_future = client->async_send_goal(request);
  if (rclcpp::spin_until_future_complete(node, goal_future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "Timed out waiting for goal acceptance; execution state is unknown" << std::endl;
    return std::nullopt;
  }
  const auto goal = goal_future.get();
  if (!goal) {
    std::cerr << "Goal rejected (controller inactive, busy, or invalid ID)" << std::endl;
    return std::nullopt;
  }
  auto future = client->async_get_result(goal);
  if (rclcpp::spin_until_future_complete(node, future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    if (rclcpp::ok()) {
      auto cancel = client->async_cancel_goal(goal);
      rclcpp::spin_until_future_complete(node, cancel, timeout);
      std::cerr << "Result wait expired; cancellation requested, waiting for final result" << std::endl;
      if (rclcpp::spin_until_future_complete(node, future, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
        return static_cast<GestureResult>(future.get().result->result);
      }
    }
    std::cerr << "No final result received; stop completion has not been confirmed" << std::endl;
    return std::nullopt;
  }
  return static_cast<GestureResult>(future.get().result->result);
}

bool report_result(
  const std::uint8_t gesture_id, const GestureResult result,
  const std::optional<GestureResult> expected_result = std::nullopt)
{
  std::cout << "gesture_id=" << static_cast<unsigned int>(gesture_id)
            << " result=" << static_cast<unsigned int>(dexhand_interfaces::to_uint8(result))
            << " (" << dexhand_interfaces::to_string(result) << ")" << std::endl;
  if (expected_result && result != *expected_result) {
    std::cerr << "Expected " << dexhand_interfaces::to_string(*expected_result) << ", got "
              << dexhand_interfaces::to_string(result) << std::endl;
    return false;
  }
  return true;
}

bool run_interactive(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<DoGesture>::SharedPtr & client,
  const std::chrono::milliseconds timeout)
{
  std::cout << "Enter a gesture ID (1-255), or q to quit." << std::endl;
  while (rclcpp::ok()) {
    std::cout << "gesture> " << std::flush;
    std::string command;
    if (!std::getline(std::cin, command)) {
      return true;
    }
    command = trim(command);
    if (command == "q" || command == "Q") {
      return true;
    }
    try {
      const auto value = parse_uint32(command, "gesture-id");
      if (value == 0 || value > 255) {
        throw std::out_of_range("gesture-id must be in [1, 255]");
      }
      const auto gesture_id = static_cast<std::uint8_t>(value);
      const auto result = call_gesture(node, client, gesture_id, timeout);
      if (!result) {
        return false;
      }
      report_result(gesture_id, *result);
    } catch (const std::exception & error) {
      std::cout << "Invalid input: " << error.what() << std::endl;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char * argv[])
{
  const auto non_ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
  Arguments args;
  if (!parse_arguments(non_ros_arguments, args)) {
    return 2;
  }
  if (args.show_help) {
    print_usage(non_ros_arguments.front().c_str());
    return 0;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dexhand_gesture_test_client");
  auto client = rclcpp_action::create_client<DoGesture>(node, args.action);
  if (!client->wait_for_action_server(args.server_wait)) {
    std::cerr << "Action server is unavailable: " << args.action << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  bool success = false;
  if (args.gesture_id) {
    const auto result = call_gesture(node, client, *args.gesture_id, args.response_timeout);
    success = result && report_result(*args.gesture_id, *result, args.expected_result);
  } else {
    success = run_interactive(node, client, args.response_timeout);
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}
