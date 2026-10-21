// Copyright 2026 botayrd.ai Robotics
//

#include "dexhand_controller/dexhand_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dexhand_controller
{
using dexhand_interfaces::DexhandMode;
using dexhand_interfaces::GestureResult;

namespace
{
// 收尾握手（下发 TOPIC+0 → hardware 复位并回报 IDLE）正常只差 1~2 个 RT 周期；
// 超过该宽限期仍未确认，说明 hardware 侧异常，强制清除占位标志，避免后续手势被永久拒绝。
constexpr auto kGestureCleanupGrace = std::chrono::milliseconds(1000);
// 执行期 BUSY feedback 的节流周期。
constexpr auto kFeedbackPeriod = std::chrono::milliseconds(100);

// 从整机版本号原始串中提取方括号内的型号（如 "[S1.2]2026.3.1-rel-a3c2e4d" -> "S1.2"）。
// 解析规则与 driver execCmdGetHandVersion() 保持一致；无括号时返回空串。
std::string parse_series_name(const std::string & version_raw)
{
  const auto open = version_raw.find('[');
  if (open == std::string::npos) {
    return {};
  }
  const auto close = version_raw.find(']', open + 1u);
  if (close == std::string::npos || close <= open + 1u) {
    return {};
  }
  return version_raw.substr(open + 1u, close - open - 1u);
}

// GoalObserver 的 ROS 适配：包住 goal handle，把纯领域结果翻译成 action 的 succeed/canceled/abort。
class GoalHandleObserver : public GoalObserver
{
public:
  GoalHandleObserver(std::shared_ptr<GestureGoalHandle> handle, rclcpp::Logger logger)
  : handle_(std::move(handle)), logger_(logger) {}

  bool is_canceling() const override
  {
    return handle_->is_canceling();
  }

  void succeeded(const GestureResult result) override
  {
    handle_->succeed(make_result(result));
  }

  void canceled(const GestureResult result) override
  {
    handle_->canceled(make_result(result));
  }

  void aborted(const GestureResult result) override
  {
    handle_->abort(make_result(result));
  }

  void busy_feedback() override
  {
    auto feedback = std::make_shared<DoGestureAction::Feedback>();
    feedback->state = dexhand_interfaces::to_uint8(GestureResult::BUSY);
    handle_->publish_feedback(feedback);
  }

private:
  std::shared_ptr<DoGestureAction::Result> make_result(const GestureResult result)
  {
    auto response = std::make_shared<DoGestureAction::Result>();
    response->result = dexhand_interfaces::to_uint8(result);
    response->description = std::string(dexhand_interfaces::to_string(result));
    RCLCPP_INFO(logger_, "Gesture finished: %s (result=%u)",
      response->description.c_str(), response->result);
    return response;
  }

  std::shared_ptr<GestureGoalHandle> handle_;
  rclcpp::Logger logger_;
};
}  // namespace

// DexhandController class
DexhandController::DexhandController()
: controller_interface::ControllerInterface(),
  tactiles_publisher_(nullptr),
  command_subscriber_(nullptr) {}


void DexhandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn DexhandController::read_parameters()
{
  params_ = param_listener_->get_params();

  command_interface_names_.clear();
  state_interface_names_.clear();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.tactiles.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'tactiles' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.gesture_gpio.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'gesture_gpio' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.gesture_result_timeout_ms <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "'gesture_result_timeout_ms' must be greater than zero");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_step_ = 3;
  least_cmd_num_ = params_.joints.size();
  least_pos_plus_tactile_num_ = least_cmd_num_ + params_.tactiles.size();

  for (const auto & joint : params_.joints) {
    for (const auto & interface : interface_names_) {
      command_interface_names_.push_back(joint + "/" + interface);
    }
  }
  for (const auto & tact : params_.tactiles) {
    // 目前指尖力控只支持法向力
    command_interface_names_.push_back(tact + "/" + "fz");
    // 指尖力检测返回: x,y轴切向力，和z轴法向力
    state_interface_names_.push_back(tact + "/" + "fx");
    state_interface_names_.push_back(tact + "/" + "fy");
    state_interface_names_.push_back(tact + "/" + "fz");
  }
  topic_command_interface_count_ = command_interface_names_.size();
  tactile_state_interface_count_ = state_interface_names_.size();
  // do_gesture gpio 的三个接口紧跟在 topic 命令 / 触觉状态之后，记下下标避免散落偏移量
  mode_interface_index_ = command_interface_names_.size();
  command_interface_names_.push_back(params_.gesture_gpio + "/mode");
  gesture_id_interface_index_ = command_interface_names_.size();
  command_interface_names_.push_back(params_.gesture_gpio + "/gesture_id");
  result_interface_index_ = state_interface_names_.size();
  state_interface_names_.push_back(params_.gesture_gpio + "/result");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandController::on_init()
{
  try {
    declare_parameters();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s",
        e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // 手势状态机与 ROS 解耦：本类只作为 ChannelPort 与 action/timer adapter 的宿主。
  const auto logger = get_node()->get_logger();
  const auto clock = get_node()->get_clock();
  // 私有继承的向上转型必须在成员函数上下文完成，不能交给 make_unique 的模板实例化。
  ChannelPort & channel_port = *this;
  session_ = std::make_unique<GestureSession>(
    GestureSessionConfig{
      std::chrono::milliseconds(params_.gesture_result_timeout_ms),
      kGestureCleanupGrace,
      kFeedbackPeriod},
    channel_port,
    [logger, clock](std::string_view message) {
      RCLCPP_WARN_THROTTLE(
        logger, *clock, 1000, "%.*s", static_cast<int>(message.size()), message.data());
    });

  callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {
      const auto cmd = *msg;

      if (!std::all_of(cmd.data.cbegin(), cmd.data.cend(),
      [](const auto & value) {return std::isfinite(value);}))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Non-finite value received. Dropping message");
        return;
      }
      std::lock_guard<std::mutex> lock(gesture_mutex_);
      if (!session_->active() || session_->claimed()) {
        // 手势期间收到的 topic 直接丢弃，不缓存到手势结束后补发。
        return;
      }
      pending_command_.topic = msg;
      command_buffer_.writeFromNonRT(pending_command_);
    }, subscription_options);

  gesture_action_ = rclcpp_action::create_server<DoGestureAction>(
    get_node(), "~/do/gesture",
    std::bind(&DexhandController::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&DexhandController::handle_cancel, this, std::placeholders::_1),
    std::bind(&DexhandController::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), callback_group_);
  gesture_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&DexhandController::poll_gesture, this),
    callback_group_);

  // Create tactile publisher
  tactiles_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/state/tactiles", rclcpp::SystemDefaultsQoS());

  // 设备身份 Service：hardware 层通过 bridge topic 发布 HandInfo（版本字符串无法经 ros2_control
  // 接口传递），此处订阅并缓存，再转成 ~/get_hand_info。独立回调组，不阻塞手势 10ms timer。
  hand_info_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions hand_info_sub_options;
  hand_info_sub_options.callback_group = hand_info_callback_group_;
  rclcpp::QoS hand_info_qos(1);
  hand_info_qos.transient_local();
  hand_info_subscriber_ = get_node()->create_subscription<dexhand_interfaces::msg::HandInfo>(
    params_.hand_info_topic, hand_info_qos,
    [this](const dexhand_interfaces::msg::HandInfo::SharedPtr msg) {
      hand_info_ = *msg;
    }, hand_info_sub_options);
  hand_info_service_ = get_node()->create_service<dexhand_interfaces::srv::GetHandInfo>(
    "~/get_hand_info",
    std::bind(&DexhandController::on_get_hand_info, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), hand_info_callback_group_);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void DexhandController::on_get_hand_info(
  const std::shared_ptr<dexhand_interfaces::srv::GetHandInfo::Request> /*request*/,
  std::shared_ptr<dexhand_interfaces::srv::GetHandInfo::Response> response)
{
  response->success = hand_info_.connected;
  response->series = hand_info_.series;
  response->version = hand_info_.version;
  response->series_name = parse_series_name(hand_info_.version);
}

// 判断是否接受手势目标；具体的接单门禁（ID 非零、已激活、无待收尾或其他任务）由 session 承担。
rclcpp_action::GoalResponse DexhandController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const DoGestureAction::Goal> goal)
{
  std::lock_guard<std::mutex> lock(gesture_mutex_);
  return session_->reserve(uuid, goal->gesture_id) ?
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse DexhandController::handle_cancel(
  const std::shared_ptr<GestureGoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(gesture_mutex_);
  // rclcpp_action 在本回调返回后才把 goal 置为 CANCELING。
  // 后续 timer 看到 is_canceling() 后发出停止命令，不能在这里提前返回 canceled。
  return session_->accept_cancel(goal_handle->get_goal_id()) ?
    rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
}

// 确认预约并启动手势；预约失效或启动失败时释放控制权并 abort 该 goal。
void DexhandController::handle_accepted(
  const std::shared_ptr<GestureGoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(gesture_mutex_);
  auto observer = std::make_shared<GoalHandleObserver>(goal_handle, get_node()->get_logger());
  if (session_->start(goal_handle->get_goal_id(), observer, std::chrono::steady_clock::now())) {
    return;
  }
  auto result = std::make_shared<DoGestureAction::Result>();
  result->result = dexhand_interfaces::to_uint8(GestureResult::FAILURE);
  result->description = std::string(dexhand_interfaces::to_string(GestureResult::FAILURE));
  goal_handle->abort(result);
}

// 10ms timer：把最近一次 hardware 观测喂给状态机，其余判定（终态/超时/收尾/反馈）都在 session 内。
void DexhandController::poll_gesture()
{
  std::lock_guard<std::mutex> lock(gesture_mutex_);
  const auto raw = gesture_result_.load(std::memory_order_acquire);
  HardwareSample sample;
  if (raw != kInvalidGestureResult) {
    sample.result = static_cast<GestureResult>(raw);
    sample.valid = true;
  }
  session_->tick(std::chrono::steady_clock::now(), sample);
}

controller_interface::InterfaceConfiguration
DexhandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DexhandController::state_interface_configuration()
const
{
  // 只发布指尖的三维力，关节状态有joint_state_broadcaster发布
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;

  return state_interfaces_config;
}

controller_interface::CallbackReturn DexhandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  bool ok = controller_interface::get_ordered_interfaces(command_interfaces_,
      command_interface_names_,
                                                         std::string(""), ordered_interfaces);
  if (!ok || command_interface_names_.size() != ordered_interfaces.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::lock_guard<std::mutex> lock(gesture_mutex_);
  // reset command buffer if a command came through callback when controller was inactive
  pending_command_ = ControllerCommand{};
  gesture_result_.store(kInvalidGestureResult, std::memory_order_release);
  // session 复位并下发 TOPIC+0，进入等 hardware 确认 IDLE 的收尾握手。
  session_->activate(std::chrono::steady_clock::now());

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(gesture_mutex_);
  // 中止在飞的手势并释放控制权（预约但未启动的 goal 会由后续 handle_accepted 兜底 abort）。
  session_->deactivate(std::chrono::steady_clock::now());
  // 停用后 update() 不再运行，直接将停止请求留在 command interface 交给 hardware。
  const auto command = session_->channel_command();
  if (!set_command_value(mode_interface_index_, dexhand_interfaces::to_uint8(command.mode)) ||
    !set_command_value(gesture_id_interface_index_, command.gesture_id))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool DexhandController::set_command_value(const std::size_t index, const double value)
{
  if (index >= command_interfaces_.size() || !command_interfaces_[index].set_value(value)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Failed to set command interface at index %zu", index);
    return false;
  }
  return true;
}

std::uint8_t DexhandController::read_gesture_result()
{
  const auto result_optional = state_interfaces_[result_interface_index_].get_optional<double>();
  const double result_value = result_optional.value_or(
    std::numeric_limits<double>::quiet_NaN());
  constexpr auto max_result = static_cast<double>(
    dexhand_interfaces::to_uint8(GestureResult::CANCELLED));
  if (std::isfinite(result_value) && std::floor(result_value) == result_value &&
    result_value >= 0.0 && result_value <= max_result)
  {
    return static_cast<std::uint8_t>(result_value);
  }
  // 无效值不能当作收尾完成的 IDLE。
  return kInvalidGestureResult;
}

controller_interface::return_type DexhandController::forward_topic_command(
  const std::shared_ptr<CmdType> & cmd_ptr)
{
  if (!cmd_ptr) {
    return controller_interface::return_type::OK;
  }

  if (cmd_ptr->data.size() == topic_command_interface_count_) {
    for (auto index = 0ul; index < topic_command_interface_count_; ++index) {
      if (!set_command_value(index, cmd_ptr->data[index])) {
        return controller_interface::return_type::ERROR;
      }
    }
    return controller_interface::return_type::OK;
  }

  if (cmd_ptr->data.size() != static_cast<std::size_t>(least_pos_plus_tactile_num_) &&
    cmd_ptr->data.size() != static_cast<std::size_t>(least_cmd_num_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) must be equal to position_only (%d), "
      "position+tactile (%d), or full_command (%zu)",
      cmd_ptr->data.size(), least_cmd_num_, least_pos_plus_tactile_num_,
      topic_command_interface_count_);
    return controller_interface::return_type::ERROR;
  }

  std::size_t input_index = 0;
  for (; input_index < static_cast<std::size_t>(least_cmd_num_); ++input_index) {
    const auto command_index = input_index * static_cast<std::size_t>(cmd_step_);
    if (!set_command_value(command_index, cmd_ptr->data[input_index]) ||
      !set_command_value(command_index + 1, default_vel_) ||
      !set_command_value(command_index + 2, default_effort_))
    {
      return controller_interface::return_type::ERROR;
    }
  }

  const bool use_default_tactile =
    cmd_ptr->data.size() == static_cast<std::size_t>(least_cmd_num_);
  auto command_index = input_index * static_cast<std::size_t>(cmd_step_);
  for (; command_index < topic_command_interface_count_; ++command_index, ++input_index) {
    const double value = use_default_tactile ? default_tactile_ : cmd_ptr->data[input_index];
    if (!set_command_value(command_index, value)) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

void DexhandController::publish_tactile_state()
{
  auto tactile_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  tactile_msg->data.reserve(tactile_state_interface_count_);
  for (auto index = 0ul; index < tactile_state_interface_count_; ++index) {
    const auto value = state_interfaces_[index].get_optional<double>();
    tactile_msg->data.push_back(value.value_or(std::numeric_limits<double>::quiet_NaN()));
  }
  tactiles_publisher_->publish(std::move(tactile_msg));
}

// ChannelPort：session 在非 RT 侧写 mailbox，update() 在 RT 侧无锁读整份快照。
void DexhandController::publish_command(const ChannelCommand & command)
{
  pending_command_.channel = command;
  if (command.claimed) {
    // Clear old topic data in the same snapshot that transfers control to the gesture.
    pending_command_.topic.reset();
  }
  command_buffer_.writeFromNonRT(pending_command_);
}

controller_interface::return_type DexhandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 把 hardware 观测发布给非 RT 侧的状态机。
  gesture_result_.store(read_gesture_result(), std::memory_order_release);

  // 一份一致快照即包含控制权与命令，无需再拼接两个独立字段。
  const auto command_ptr = command_buffer_.readFromRT();
  const ControllerCommand snapshot = command_ptr ? *command_ptr : ControllerCommand{};
  const auto & command = snapshot.channel;
  if (!set_command_value(mode_interface_index_, dexhand_interfaces::to_uint8(command.mode)) ||
    !set_command_value(gesture_id_interface_index_, command.gesture_id))
  {
    return controller_interface::return_type::ERROR;
  }

  if (!command.claimed &&
    forward_topic_command(snapshot.topic) != controller_interface::return_type::OK)
  {
    return controller_interface::return_type::ERROR;
  }

  publish_tactile_state();
  return controller_interface::return_type::OK;
}


}  // namespace dexhand_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dexhand_controller::DexhandController, controller_interface::ControllerInterface)
