// Copyright 2026 botayrd.ai Robotics
//

#ifndef DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_
#define DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "dexhand_controller/dexhand_controller_parameters.hpp"
#include "dexhand_controller/gesture_session.hpp"
#include "dexhand_interfaces/gesture_enums.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "dexhand_interfaces/action/do_gesture.hpp"

namespace dexhand_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;
using DoGestureAction = dexhand_interfaces::action::DoGesture;
using GestureGoalHandle = rclcpp_action::ServerGoalHandle<DoGestureAction>;

/**
 * \brief Dexhand controller for a set of interfaces.
 *
 * This class forwards the command signal down to a set of interfaces on the specified joint/tactile sensor.
 *
 * \param joints Names of the joints to control.
 * \param interface_names Names of the interfaces to command.
 * \param tactiles Names of the tactile sensors to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 *
 * 手势生命周期（接单/启动/取消/收尾/超时/握手）由 GestureSession 承载；本类只做 ROS 适配：
 * action 回调 / 10ms timer / lifecycle 与无锁 mailbox 的翻译，并实现 ChannelPort 供 session 下发。
 */
class DexhandController : public controller_interface::ControllerInterface,
                          private ChannelPort
{
public:
  DexhandController();

  ~DexhandController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  void declare_parameters();

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_names_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  controller_interface::CallbackReturn read_parameters();

private:
  static constexpr std::uint8_t kInvalidGestureResult = 255;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DoGestureAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GestureGoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GestureGoalHandle> goal_handle);

  void poll_gesture();
  bool set_command_value(std::size_t index, double value);
  std::uint8_t read_gesture_result();
  controller_interface::return_type forward_topic_command(const std::shared_ptr<CmdType> & command);
  void publish_tactile_state();
  // ChannelPort：GestureSession 下发一致快照的唯一出口（非 RT 写，RT 读）
  void publish_command(const ChannelCommand & command) override;

  std::vector<std::string> interface_names_ = {"position", "velocity", "effort"};

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  const double default_vel_ = 1.5708;
  const double default_effort_ = 2000.0;
  const double default_tactile_ = 100.0;
  int cmd_step_ = 0;
  int least_cmd_num_ = 0;  // 纯位置控制时，命令的最小长度；
  int least_pos_plus_tactile_num_ = 0;  // 位置控制 + 指尖力控制时，命令的最小长度
  std::size_t topic_command_interface_count_ = 0;
  std::size_t tactile_state_interface_count_ = 0;
  // do_gesture gpio 三个接口的下标，避免在下发/回读处散落 "+1" 这类偏移量
  std::size_t mode_interface_index_ = 0;
  std::size_t gesture_id_interface_index_ = 0;
  std::size_t result_interface_index_ = 0;

  struct ControllerCommand
  {
    ChannelCommand channel;
    std::shared_ptr<CmdType> topic;
  };

  ControllerCommand pending_command_;
  realtime_tools::RealtimeBuffer<ControllerCommand> command_buffer_;
  rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tactiles_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // hardware 观测：update() 写、timer 读
  std::atomic<std::uint8_t> gesture_result_{kInvalidGestureResult};

  // 手势状态与 pending_command_ 均由 gesture_mutex_ 串行化。
  std::unique_ptr<GestureSession> session_;
  std::mutex gesture_mutex_;
  rclcpp_action::Server<DoGestureAction>::SharedPtr gesture_action_;
  rclcpp::TimerBase::SharedPtr gesture_timer_;

  // DexhandController specific parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace dexhand_controller

#endif  // DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_
