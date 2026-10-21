// Copyright 2026 botyard.ai Robotics
//
// EtherCAT 链路 ROS 集成测试。
// 本程序是纯 ROS 客户端，不直接依赖 dexhand_driver：
//   - EtherCAT 连接由 ros2_control 的 DexhandHardwareInterface(communicate:=EtherCAT) 负责；
//   - 关节状态读 /joint_states，关节与指尖力命令写 dexhand_controller 的 ~/commands，
//     触觉状态读 dexhand_controller 的 ~/state/tactiles，手势调用 dexhand_controller 的 ~/do/gesture，
//     硬件接口发布的网格触觉读 ~/grid_tactiles。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "dexhand_interfaces/gesture_enums.hpp"
#include "dexhand_interfaces/msg/tactile_grid.hpp"
#include "dexhand_interfaces/action/do_gesture.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

using DoGesture = dexhand_interfaces::action::DoGesture;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using JointState = sensor_msgs::msg::JointState;
using TactileGrid = dexhand_interfaces::msg::TactileGrid;
using GestureResult = dexhand_interfaces::GestureResult;

// 与 dexhand_hardware_interface on_activate() 中 `~/grid_tactiles` 的默认解析结果一致
constexpr char kDefaultGridTactileTopic[] = "/botyard_dexhand_hardware/grid_tactiles";
constexpr char kDefaultController[] = "dexhand_controller";
constexpr std::size_t kFingerCount = 5;
const char * const kFingerNames[kFingerCount] = {"THJ", "FFJ", "MFJ", "RFJ", "LFJ"};

// 兜底配置，与 dexhand_bringup/config/dexhand_controller.yaml 保持一致；
// 正常情况下会优先从控制器节点参数里读取。
const std::vector<std::string> kFallbackJoints = {
  "FAJ3", "FAJ1", "THJ4", "THJ3", "THJ2", "THJ1", "FFJ4", "FFJ3", "FFJ2",
  "MFJ4", "MFJ3", "MFJ2", "RFJ4", "RFJ3", "RFJ2", "LFJ4", "LFJ3", "LFJ2"};
const std::vector<std::string> kFallbackTactiles = {"THT0", "FFT0", "MFT0", "RFT0", "LFT0"};

struct Options
{
  std::string ns;
  std::string controller = kDefaultController;
  std::string grid_tactile_topic = kDefaultGridTactileTopic;
  bool show_help = false;
};

struct Context
{
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::string ns;
  std::string controller;
  std::string grid_tactile_topic;

  std::vector<std::string> joints;     // 顺序必须与控制器 joints 参数一致
  std::vector<std::string> tactiles;   // 顺序必须与控制器 tactiles 参数一致
  bool config_from_controller = false;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr tactile_state_sub;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr command_pub;
  rclcpp_action::Client<DoGesture>::SharedPtr gesture_client;

  JointState::ConstSharedPtr joint_state;
  Float64MultiArray::ConstSharedPtr tactile_state;
  uint64_t joint_state_seq = 0;
  uint64_t tactile_state_seq = 0;

  std::string joint_states_topic() const { return resolve("joint_states"); }
  std::string commands_topic() const { return resolve(controller + "/commands"); }
  std::string tactiles_topic() const { return resolve(controller + "/state/tactiles"); }
  std::string gesture_service() const { return resolve(controller + "/do/gesture"); }
  std::string controller_node() const { return resolve(controller); }

  std::string resolve(const std::string & relative) const
  {
    if (ns.empty()) {
      return "/" + relative;
    }
    return "/" + ns + "/" + relative;
  }
};

// ---------------------------------------------------------------------------
// 字符串与解析工具
// ---------------------------------------------------------------------------

std::string trim(const std::string & text)
{
  std::size_t begin = 0;
  while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  std::size_t end = text.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::string toLower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

std::string toUpper(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return text;
}

bool parseUnsigned(const std::string & text, unsigned long maximum, unsigned long * value)
{
  std::istringstream stream(text);
  std::string trailing;
  unsigned long parsed = 0;
  if (!(stream >> parsed) || stream >> trailing || parsed > maximum) {
    return false;
  }
  *value = parsed;
  return true;
}

bool parseDouble(const std::string & text, double * value)
{
  std::istringstream stream(text);
  std::string trailing;
  double parsed = 0.0;
  if (!(stream >> parsed) || stream >> trailing) {
    return false;
  }
  *value = parsed;
  return true;
}

// ---------------------------------------------------------------------------
// 自旋与等待
// ---------------------------------------------------------------------------

class Spinner
{
public:
  Spinner(
    const rclcpp::Node::SharedPtr & node,
    const rclcpp::executors::SingleThreadedExecutor::SharedPtr & executor)
  : node_(node), executor_(executor) {}

  void spinOnce() { executor_->spin_some(); }

  bool waitFor(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      spinOnce();
      if (predicate()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

// 等待消息数据到达 / 刷新。返回 false 表示超时。
bool refreshJointState(Context & context, Spinner & spinner)
{
  const uint64_t previous = context.joint_state_seq;
  if (!spinner.waitFor(
      [&context, previous]() {return context.joint_state_seq != previous;},
      std::chrono::milliseconds(2000)))
  {
    std::cerr << "未收到 " << context.joint_states_topic()
              << " 数据：确认 ros2_control 已启动且 joint_state_broadcaster 处于 active。\n";
    return false;
  }
  return true;
}

bool refreshTactileState(Context & context, Spinner & spinner)
{
  const uint64_t previous = context.tactile_state_seq;
  if (!spinner.waitFor(
      [&context, previous]() {return context.tactile_state_seq != previous;},
      std::chrono::milliseconds(2000)))
  {
    std::cerr << "未收到 " << context.tactiles_topic()
              << " 数据：确认 " << context.controller << " 控制器处于 active。\n";
    return false;
  }
  return true;
}

double jointStateValue(
  const Context & context, const std::string & joint,
  const std::vector<double> JointState::* field)
{
  const JointState & state = *context.joint_state;
  const std::vector<double> & values = state.*field;
  for (std::size_t index = 0; index < state.name.size() && index < values.size(); ++index) {
    if (state.name[index] == joint) {
      return values[index];
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void printJointStates(const Context & context, const std::vector<std::string> & selected)
{
  std::cout << "  关节      position(rad)  velocity(rad/s)  effort\n";
  for (const std::string & joint : selected) {
    const double position = jointStateValue(context, joint, &JointState::position);
    const double velocity = jointStateValue(context, joint, &JointState::velocity);
    const double effort = jointStateValue(context, joint, &JointState::effort);
    std::cout << "  " << std::left << std::setw(8) << joint << std::right << std::fixed
              << std::setprecision(4) << std::setw(14) << position << std::setw(17) << velocity
              << std::setw(9) << effort << '\n';
  }
  std::cout << std::defaultfloat;
}

void printTactileStates(const Context & context)
{
  const std::vector<std::string> & tactiles = context.tactiles;
  const Float64MultiArray & state = *context.tactile_state;
  std::cout << "  触觉       fx      fy      fz\n";
  for (std::size_t index = 0; index < tactiles.size(); ++index) {
    const std::size_t base = index * 3u;
    if (base + 2u >= state.data.size()) {
      std::cout << "  " << std::left << std::setw(8) << tactiles[index] << std::right
                << "  （数据长度不足: " << state.data.size() << "）\n";
      continue;
    }
    std::cout << "  " << std::left << std::setw(8) << tactiles[index] << std::right << std::fixed
              << std::setprecision(1) << std::setw(8) << state.data[base] << std::setw(8)
              << state.data[base + 1u] << std::setw(8) << state.data[base + 2u] << '\n';
  }
  std::cout << std::defaultfloat;
}

// ---------------------------------------------------------------------------
// 选择与输入
// ---------------------------------------------------------------------------

void printNameTable(const std::vector<std::string> & names)
{
  for (std::size_t index = 0; index < names.size(); ++index) {
    std::cout << "  " << std::setw(2) << index + 1u << ": " << std::left << std::setw(8)
              << names[index] << std::right;
    if ((index + 1u) % 4u == 0u || index + 1u == names.size()) {
      std::cout << '\n';
    }
  }
}

bool parseSelection(
  const std::string & input,
  const std::vector<std::string> & available,
  std::vector<std::string> * selected)
{
  selected->clear();
  std::string normalized = input;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream stream(normalized);
  std::string token;
  while (stream >> token) {
    if (toLower(token) == "all") {
      *selected = available;
      return true;
    }

    std::string name;
    unsigned long index = 0;
    if (parseUnsigned(token, available.size(), &index) && index > 0u) {
      name = available[index - 1u];
    } else {
      const std::string candidate = toUpper(token);
      const auto found = std::find(available.begin(), available.end(), candidate);
      if (found == available.end()) {
        return false;
      }
      name = *found;
    }
    if (std::find(selected->begin(), selected->end(), name) == selected->end()) {
      selected->push_back(name);
    }
  }
  return !selected->empty();
}

bool promptSelection(
  const char * kind,
  const std::vector<std::string> & available,
  const std::vector<std::string> & defaults,
  std::vector<std::string> * selected)
{
  std::cout << "可选" << kind << ":\n";
  printNameTable(available);
  while (true) {
    std::cout << "请输入" << kind << "序号或名称，多个用空格/逗号分隔，all 表示全部"
              << (defaults.empty() ? "" : "，直接回车使用默认项") << "（q 取消）: " << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) {
      return false;
    }
    input = trim(input);
    if (toLower(input) == "q") {
      return false;
    }
    if (input.empty() && !defaults.empty()) {
      *selected = defaults;
      return true;
    }
    if (parseSelection(input, available, selected)) {
      return true;
    }
    std::cout << "选择无效，请重新输入。\n";
  }
}

// 目标位置默认取当前反馈，回车即保持不变
bool promptJointTargets(
  const Context & context, const std::vector<std::string> & selected,
  std::vector<double> * targets)
{
  for (const std::string & joint : selected) {
    const auto found = std::find(context.joints.begin(), context.joints.end(), joint);
    if (found == context.joints.end()) {
      std::cerr << "关节 " << joint << " 不在控制器 joints 参数中\n";
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(found - context.joints.begin());
    const double current = jointStateValue(context, joint, &JointState::position);
    double target = current;
    while (true) {
      std::cout << joint << " 当前 position=" << std::fixed << std::setprecision(4) << current
                << " rad\n  输入目标 position（回车=" << target << "，q 取消）: " << std::defaultfloat
                << std::flush;
      std::string input;
      if (!std::getline(std::cin, input)) {
        return false;
      }
      input = trim(input);
      if (toLower(input) == "q") {
        return false;
      }
      if (input.empty() || parseDouble(input, &target)) {
        break;
      }
      std::cout << "请输入一个浮点数。\n";
    }
    (*targets)[index] = target;
  }
  return true;
}

// 指尖力目标，回车表示 0（硬件侧 fz < 1.0 视为不控力）
bool promptTactileTargets(const std::vector<std::string> & selected, std::vector<double> * targets)
{
  for (const std::string & tactile : selected) {
    const auto found = std::find(kFallbackTactiles.begin(), kFallbackTactiles.end(), tactile);
    if (found == kFallbackTactiles.end()) {
      std::cerr << "触觉 " << tactile << " 不在支持列表中\n";
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(found - kFallbackTactiles.begin());
    double target = 0.0;
    while (true) {
      std::cout << tactile << " 输入目标 fz（回车=0 即不控力，q 取消）: " << std::flush;
      std::string input;
      if (!std::getline(std::cin, input)) {
        return false;
      }
      input = trim(input);
      if (toLower(input) == "q") {
        return false;
      }
      if (input.empty() || parseDouble(input, &target)) {
        break;
      }
      std::cout << "请输入一个浮点数。\n";
    }
    (*targets)[index] = target;
  }
  return true;
}

std::vector<double> currentJointPositions(const Context & context)
{
  std::vector<double> positions(context.joints.size(), 0.0);
  for (std::size_t index = 0; index < context.joints.size(); ++index) {
    const double value = jointStateValue(context, context.joints[index], &JointState::position);
    positions[index] = std::isfinite(value) ? value : 0.0;
  }
  return positions;
}

void publishCommand(Context & context, const std::vector<double> & data, const char * form)
{
  Float64MultiArray message;
  message.data = data;
  context.command_pub->publish(message);
  std::cout << "已发布 " << context.commands_topic() << "（长度 " << data.size() << "，" << form
            << "）\n";
}

// ---------------------------------------------------------------------------
// 1) getDofs：读 /joint_states
// ---------------------------------------------------------------------------

bool runGetDofs(Context & context, Spinner & spinner)
{
  if (!refreshJointState(context, spinner)) {
    return false;
  }
  std::vector<std::string> selected;
  if (!promptSelection("关节", context.joints, context.joints, &selected)) {
    std::cout << "getDofs 已取消\n";
    return true;
  }
  std::cout << "getDofs（来自 " << context.joint_states_topic() << "）:\n";
  printJointStates(context, selected);
  return true;
}

// ---------------------------------------------------------------------------
// 2) setDofs：写 ~/commands（position-only 形式，长度 = 关节数）
// ---------------------------------------------------------------------------

bool runSetDofs(Context & context, Spinner & spinner)
{
  if (!refreshJointState(context, spinner)) {
    return false;
  }
  std::vector<std::string> selected;
  if (!promptSelection("关节", context.joints, context.joints, &selected)) {
    std::cout << "setDofs 已取消\n";
    return true;
  }

  std::vector<double> targets = currentJointPositions(context);
  if (!promptJointTargets(context, selected, &targets)) {
    std::cout << "setDofs 已取消\n";
    return true;
  }
  publishCommand(context, targets, "position-only");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (refreshJointState(context, spinner)) {
    std::cout << "发送后的关节状态:\n";
    printJointStates(context, selected);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 3) getDofsAndTactiles：读 /joint_states + ~/state/tactiles
// ---------------------------------------------------------------------------

bool runGetDofsAndTactiles(Context & context, Spinner & spinner)
{
  if (!refreshJointState(context, spinner) || !refreshTactileState(context, spinner)) {
    return false;
  }
  std::vector<std::string> selected;
  if (!promptSelection("关节", context.joints, context.joints, &selected)) {
    std::cout << "getDofsAndTactiles 已取消\n";
    return true;
  }
  std::cout << "getDofsAndTactiles 关节状态（来自 " << context.joint_states_topic() << "）:\n";
  printJointStates(context, selected);
  std::cout << "指尖触觉（来自 " << context.tactiles_topic() << "，顺序 fx/fy/fz）:\n";
  printTactileStates(context);
  return true;
}

// ---------------------------------------------------------------------------
// 4) setDofsAndTactiles：写 ~/commands（position+tactile 形式，长度 = 关节数 + 触觉数）
// ---------------------------------------------------------------------------

bool runSetDofsAndTactiles(Context & context, Spinner & spinner)
{
  if (!refreshJointState(context, spinner) || !refreshTactileState(context, spinner)) {
    return false;
  }
  std::vector<std::string> selectedJoints;
  if (!promptSelection("关节", context.joints, context.joints, &selectedJoints)) {
    std::cout << "setDofsAndTactiles 已取消\n";
    return true;
  }
  std::vector<double> targets = currentJointPositions(context);
  if (!promptJointTargets(context, selectedJoints, &targets)) {
    std::cout << "setDofsAndTactiles 已取消\n";
    return true;
  }

  std::vector<std::string> selectedTactiles;
  std::cout << "不选择的指尖按 fz=0（硬件侧 fz < 1.0 视为不控力）处理。\n";
  if (!promptSelection("触觉", context.tactiles, {}, &selectedTactiles)) {
    std::cout << "setDofsAndTactiles 已取消\n";
    return true;
  }
  std::vector<double> tactileTargets(context.tactiles.size(), 0.0);
  if (!promptTactileTargets(selectedTactiles, &tactileTargets)) {
    std::cout << "setDofsAndTactiles 已取消\n";
    return true;
  }

  std::vector<double> data = targets;
  data.insert(data.end(), tactileTargets.begin(), tactileTargets.end());
  publishCommand(context, data, "position+tactile");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (refreshJointState(context, spinner) && refreshTactileState(context, spinner)) {
    std::cout << "发送后的关节状态:\n";
    printJointStates(context, selectedJoints);
    std::cout << "发送后的指尖触觉:\n";
    printTactileStates(context);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 5) doGesture：调用 dexhand_controller 的 ~/do/gesture
// ---------------------------------------------------------------------------

bool runDoGesture(Context & context, Spinner & spinner)
{
  if (!context.gesture_client->wait_for_action_server(std::chrono::seconds(3))) {
    std::cerr << "Action server 不可用: " << context.gesture_service()
              << "（确认 " << context.controller << " 控制器处于 active）\n";
    return false;
  }

  std::cout << "动作 ID: 1 五指握, 2 四指握, 3 三指握, 4 四指扣握, "
               "5 五指扣握, 6 二指捏合, 7 三指捏合, 8 爪式, "
               "9 夹式, 10 钳式, 255 释放/张开\n"
            << "请输入动作 ID（1-255，q 取消）: " << std::flush;
  std::string input;
  if (!std::getline(std::cin, input) || toLower(trim(input)) == "q") {
    std::cout << "doGesture 已取消\n";
    return true;
  }
  unsigned long gesture_id = 0;
  if (!parseUnsigned(trim(input), 255, &gesture_id) || gesture_id == 0u) {
    std::cout << "无效动作 ID。\n";
    return true;
  }

  DoGesture::Goal request;
  request.gesture_id = static_cast<uint8_t>(gesture_id);
  auto accepted = context.gesture_client->async_send_goal(request);
  if (!spinner.waitFor(
      [&accepted]() {return accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;},
      std::chrono::milliseconds(15000))) {
    std::cerr << "等待动作接单超时，执行状态未知。\n";
    return false;
  }
  const auto goal = accepted.get();
  if (!goal) {
    std::cerr << "动作被拒绝（控制器未激活、忙或 ID 非法）。\n";
    return false;
  }
  auto future = context.gesture_client->async_get_result(goal);
  const auto ready = [&future]() {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    };
  if (!spinner.waitFor(ready, std::chrono::milliseconds(15000))) {
    auto cancel = context.gesture_client->async_cancel_goal(goal);
    spinner.waitFor(
      [&cancel]() {return cancel.wait_for(std::chrono::seconds(0)) == std::future_status::ready;},
      std::chrono::milliseconds(3000));
    if (!spinner.waitFor(ready, std::chrono::milliseconds(15000))) {
      std::cerr << "已请求取消，但尚未确认底层收尾完成。\n";
      return false;
    }
  }
  const auto response = future.get().result;
  const auto result = static_cast<GestureResult>(response->result);
  std::cout << "动作 result=" << static_cast<unsigned>(response->result) << " ("
            << dexhand_interfaces::to_string(result) << ")\n";
  return result == GestureResult::SUCCESS;
}

// ---------------------------------------------------------------------------
// 6) publish_grid_tactiles：订阅硬件接口发布的网格触觉话题
// ---------------------------------------------------------------------------

// 一秒窗口内的订阅统计，回调与打印都在同一线程（executor->spin_some()），无需加锁。
struct GridTactileWindow
{
  uint64_t total = 0;                  // 累计收到的消息数
  uint64_t messages = 0;               // 本窗口接收条数
  uint64_t nonzero_points = 0;         // 本窗口非零点数
  std::array<int, kFingerCount> max_abs_fz{};
  // 用消息时间戳估算发布频率：接收条数会被本进程轮询节奏限制，时间戳不会
  uint64_t stamped = 0;
  int64_t stamp_first_ns = 0;
  int64_t stamp_last_ns = 0;
  std::string sample_frame_id;
  std::size_t sample_points = 0;
  uint16_t sample_point_id = 0;
  int8_t sample_fx = 0;
};

bool runSubscribeGridTactiles(Context & context, Spinner & spinner)
{
  std::cout << "==> 订阅网格触觉话题: " << context.grid_tactile_topic << " (SensorDataQoS)\n"
            << "    数据由 dexhand_hardware_interface 的 publish_grid_tactiles() 发布，"
               "需 ros2_control 以 communicate:=EtherCAT 激活。\n"
            << "    按回车停止订阅并返回菜单。\n";

  GridTactileWindow window;
  bool first_message = true;
  auto subscription = context.node->create_subscription<TactileGrid>(
    context.grid_tactile_topic, rclcpp::SensorDataQoS(),
    [&window, &first_message](TactileGrid::ConstSharedPtr message) {
      ++window.total;
      ++window.messages;
      const int64_t stamp_ns =
        static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(message->header.stamp.nanosec);
      if (stamp_ns > 0) {
        if (window.stamped == 0) {
          window.stamp_first_ns = stamp_ns;
        }
        window.stamp_last_ns = stamp_ns;
        ++window.stamped;
      }
      for (std::size_t finger = 0; finger < message->fingers.size(); ++finger) {
        int finger_max_abs_fz = 0;
        for (const auto & point : message->fingers[finger].points) {
          if (point.fx != 0 || point.fy != 0 || point.fz != 0) {
            ++window.nonzero_points;
          }
          finger_max_abs_fz = std::max(
            finger_max_abs_fz, std::abs(static_cast<int>(point.fz)));
        }
        if (finger < window.max_abs_fz.size()) {
          window.max_abs_fz[finger] = std::max(window.max_abs_fz[finger], finger_max_abs_fz);
        }
      }
      if (first_message) {
        first_message = false;
        window.sample_frame_id = message->header.frame_id;
        window.sample_points = message->fingers[0].points.size();
        window.sample_point_id = message->fingers[0].points[0].point_id;
        window.sample_fx = message->fingers[0].points[0].fx;
      }
    });

  std::atomic<bool> stop{false};
  std::thread input_thread([&stop]() {
      std::string line;
      std::getline(std::cin, line);
      stop = true;
    });

  auto window_start = std::chrono::steady_clock::now();
  unsigned int idle_windows = 0;
  while (!stop.load() && rclcpp::ok()) {
    spinner.spinOnce();
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - window_start).count();
    if (elapsed >= 1.0) {
      const bool hz_from_stamp =
        window.stamped >= 2 && window.stamp_last_ns > window.stamp_first_ns;
      const double publish_hz = hz_from_stamp ?
        static_cast<double>(window.stamped - 1) * 1e9 /
        static_cast<double>(window.stamp_last_ns - window.stamp_first_ns) :
        static_cast<double>(window.messages) / elapsed;

      std::ostringstream line;
      line << std::fixed << std::setprecision(1)
           << "  发布频率=" << publish_hz << "Hz" << (hz_from_stamp ? "(时间戳)" : "(接收估算)")
           << " 接收=" << static_cast<uint64_t>(static_cast<double>(window.messages) / elapsed)
           << "条/s 累计=" << window.total
           << " 本秒非零点=" << window.nonzero_points << " 各指max|fz|:";
      for (std::size_t finger = 0; finger < window.max_abs_fz.size(); ++finger) {
        line << ' ' << kFingerNames[finger] << '=' << window.max_abs_fz[finger];
      }
      std::cout << line.str() << std::endl;

      if (window.messages == 0) {
        ++idle_windows;
        if (idle_windows == 3u) {
          std::cout << "  [提示] 已连续 3 秒未收到消息：请确认 ros2_control 已用 "
                       "communicate:=EtherCAT 激活，且话题名与 " << context.grid_tactile_topic
                    << " 一致（ros2 topic list）。\n";
        }
      } else {
        idle_windows = 0;
      }

      window_start = now;
      window.messages = 0;
      window.nonzero_points = 0;
      window.max_abs_fz.fill(0);
      window.stamped = 0;
      window.stamp_first_ns = 0;
      window.stamp_last_ns = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  input_thread.join();
  subscription.reset();

  std::cout << "已停止订阅，累计收到 " << window.total << " 条消息。\n";
  if (window.total > 0) {
    std::cout << "  首帧示例: frame_id=" << window.sample_frame_id
              << " points/finger=" << window.sample_points
              << " fingers[0].points[0]={point_id=" << window.sample_point_id
              << ", fx=" << static_cast<int>(window.sample_fx) << "}\n";
  }
  return window.total > 0;
}

// ---------------------------------------------------------------------------
// 控制器配置发现
// ---------------------------------------------------------------------------

bool discoverControllerConfig(Context & context)
{
  context.joints = kFallbackJoints;
  context.tactiles = kFallbackTactiles;
  context.config_from_controller = false;

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    context.node, context.controller_node());
  if (!parameters_client->wait_for_service(std::chrono::seconds(3))) {
    std::cerr << "[警告] 读不到 " << context.controller_node()
              << " 参数：该控制器节点不在当前 ROS 图里。按以下顺序排查：\n"
              << "        1) 控制器是否已加载（ros2 control list_controllers）；"
                 "EtherCAT 连不上时 ros2_control_node 会在启动阶段就崩，"
                 "此时 pgrep -f ros2_control_node 为空；\n"
              << "        2) 节点属主是否为当前用户"
                 "（ps -o user= -p $(pgrep -f ros2_control_node)）：若为 root，"
                 "说明 launch 是用 sudo 启动的（setpriv 降权失效），"
                 "与终端不同用户，Fast DDS 共享内存不通；\n"
              << "        3) 能力是否带上（getpcaps $(pgrep -f ros2_control_node)）："
                 "SOEM 需要 CAP_NET_RAW 与 CAP_NET_ADMIN，"
                 "由 ros2_control.launch.py 里的 setpriv 前缀授予。\n"
              << "        改用内置默认关节/触觉顺序，发送非零位置前请先核对顺序。\n";
    return false;
  }

  try {
    const auto joints = parameters_client->get_parameter(
      "joints", std::vector<std::string>{});
    const auto tactiles = parameters_client->get_parameter(
      "tactiles", std::vector<std::string>{});
    if (!joints.empty()) {
      context.joints = joints;
    }
    if (!tactiles.empty()) {
      context.tactiles = tactiles;
    }
    if (!joints.empty() && !tactiles.empty()) {
      context.config_from_controller = true;
      std::cout << "已从 " << context.controller_node() << " 读取配置: " << context.joints.size()
                << " 个关节, " << context.tactiles.size() << " 个触觉。\n";
      return true;
    }
    std::cerr << "[警告] 控制器参数 joints/tactiles 为空，改用内置默认顺序"
      "（可用 ros2 param list " << context.controller_node() << " 核对）。\n";
  } catch (const std::exception & error) {
    std::cerr << "[警告] 读取控制器参数失败: " << error.what() << "，改用内置默认顺序。\n";
  }
  return false;
}

// ---------------------------------------------------------------------------
// 菜单
// ---------------------------------------------------------------------------

void printMenu(const Context & context)
{
  std::cout << "\n========== dexhand EtherCAT ROS 集成测试菜单 ==========\n"
            << "  1 - getDofs               读 " << context.joint_states_topic() << "\n"
            << "  2 - setDofs               写 " << context.commands_topic()
            << "（position-only）\n"
            << "  3 - getDofsAndTactiles    读 " << context.joint_states_topic() << " + "
            << context.tactiles_topic() << "\n"
            << "  4 - setDofsAndTactiles    写 " << context.commands_topic()
            << "（position+tactile）\n"
            << "  5 - doGesture             调 " << context.gesture_service() << "\n"
            << "  6 - publish_grid_tactiles 订阅 " << context.grid_tactile_topic << "\n"
            << "  h - 显示菜单\n"
            << "  q - 退出\n";
  if (!context.config_from_controller) {
    std::cout << "  [注意] 当前使用内置默认关节/触觉顺序，未从控制器参数确认。\n";
  }
}

void printUsage(const char * program)
{
  std::cout << "用法:\n"
            << "  " << program << " [options]\n\n"
            << "选项:\n"
            << "  --namespace NS         控制器管理器命名空间（默认空）\n"
            << "  --controller NAME      控制器名（默认 " << kDefaultController << "）\n"
            << "  --grid-topic NAME      网格触觉话题（默认 " << kDefaultGridTactileTopic << "）\n"
            << "  -h, --help             显示帮助\n\n"
            << "说明:\n"
            << "  本程序是纯 ROS 客户端，不连接 EtherCAT；\n"
            << "  请先用带 communicate:=EtherCAT 的 launch 启动 ros2_control，再运行本程序。\n";
}

std::string takeValue(
  const std::vector<std::string> & arguments, std::size_t & index, const char * option)
{
  if (++index >= arguments.size()) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return arguments[index];
}

bool parseArguments(const std::vector<std::string> & arguments, Options & options)
{
  try {
    for (std::size_t index = 1; index < arguments.size(); ++index) {
      const std::string & argument = arguments[index];
      if (argument == "-h" || argument == "--help") {
        options.show_help = true;
        return true;
      } else if (argument == "--namespace") {
        options.ns = takeValue(arguments, index, "--namespace");
      } else if (argument == "--controller") {
        options.controller = takeValue(arguments, index, "--controller");
      } else if (argument == "--grid-topic") {
        options.grid_tactile_topic = takeValue(arguments, index, "--grid-topic");
      } else {
        throw std::invalid_argument("unknown argument: " + argument);
      }
    }
    while (!options.ns.empty() && options.ns.front() == '/') {
      options.ns.erase(options.ns.begin());
    }
    while (!options.ns.empty() && options.ns.back() == '/') {
      options.ns.pop_back();
    }
    if (options.controller.empty()) {
      throw std::invalid_argument("controller must not be empty");
    }
    if (options.grid_tactile_topic.empty()) {
      throw std::invalid_argument("grid topic must not be empty");
    }
  } catch (const std::exception & error) {
    std::cerr << "参数错误: " << error.what() << '\n';
    printUsage(arguments.front().c_str());
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const std::vector<std::string> non_ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
    Options options;
    if (!parseArguments(non_ros_arguments, options)) {
      return 2;
    }
    if (options.show_help) {
      printUsage(non_ros_arguments.front().c_str());
      return 0;
    }

    rclcpp::init(argc, argv);

    Context context;
    context.ns = options.ns;
    context.controller = options.controller;
    context.grid_tactile_topic = options.grid_tactile_topic;
    context.node = std::make_shared<rclcpp::Node>("dexhand_ethercat_test");
    context.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // SyncParametersClient 内部会自旋本节点，此时节点不能已挂在别的 executor 上
    discoverControllerConfig(context);

    context.executor->add_node(context.node);

    // 状态订阅用 best-effort，兼容 reliable / best-effort 两种发布端
    context.joint_state_sub = context.node->create_subscription<JointState>(
      context.joint_states_topic(), rclcpp::SensorDataQoS(),
      [&context](JointState::ConstSharedPtr message) {
        context.joint_state = message;
        ++context.joint_state_seq;
      });
    context.tactile_state_sub = context.node->create_subscription<Float64MultiArray>(
      context.tactiles_topic(), rclcpp::SensorDataQoS(),
      [&context](Float64MultiArray::ConstSharedPtr message) {
        context.tactile_state = message;
        ++context.tactile_state_seq;
      });
    // 控制器以 SystemDefaultsQoS(reliable) 订阅，发布端保持一致
    context.command_pub = context.node->create_publisher<Float64MultiArray>(
      context.commands_topic(), rclcpp::SystemDefaultsQoS());
    context.gesture_client = rclcpp_action::create_client<DoGesture>(context.node, context.gesture_service());

    Spinner spinner(context.node, context.executor);
    printMenu(context);

    while (true) {
      std::cout << "\n请输入操作: " << std::flush;
      std::string command;
      if (!std::getline(std::cin, command)) {
        break;
      }
      command = toLower(trim(command));
      if (command == "q" || command == "quit" || command == "exit") {
        break;
      }
      if (command == "h" || command == "help") {
        printMenu(context);
      } else if (command == "1" || command == "getdofs") {
        runGetDofs(context, spinner);
      } else if (command == "2" || command == "setdofs") {
        runSetDofs(context, spinner);
      } else if (command == "3" || command == "getdofsandtactiles") {
        runGetDofsAndTactiles(context, spinner);
      } else if (command == "4" || command == "setdofsandtactiles") {
        runSetDofsAndTactiles(context, spinner);
      } else if (command == "5" || command == "dogesture") {
        runDoGesture(context, spinner);
      } else if (command == "6" || command == "publishgridtactiles" ||
        command == "subscribegridtactiles")
      {
        runSubscribeGridTactiles(context, spinner);
      } else if (!command.empty()) {
        std::cout << "未知操作，输入 h 查看菜单。\n";
      }
    }

    context.executor->remove_node(context.node);
    context.node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "EtherCAT 测试异常: " << error.what() << '\n';
    return 1;
  }
}
