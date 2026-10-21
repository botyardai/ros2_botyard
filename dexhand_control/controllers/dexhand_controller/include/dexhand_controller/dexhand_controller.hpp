// Copyright 2026 botayrd.ai Robotics
//

#ifndef DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_
#define DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
// #include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "dexhand_controller/dexhand_controller_parameters.hpp"

namespace dexhand_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;

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
 */
class DexhandController : public controller_interface::ControllerInterface
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

  std::vector<std::string> joint_names_;
  std::vector<std::string> interface_names_ = {"position", "velocity", "effort"};
  // std::vector<std::string> tactile_names_ = {"THT0", "FFT0", "MFT0", "RFT0", "LFT0"};

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  const double default_vel_ = 1.5708;
  const double default_effort_ = 2000.0;
  const double default_tactile_ = 100.0;
  int cmd_step_ = 0;
  int least_cmd_num_ = 0;  // 纯位置控制时，命令的最小长度；
  int least_pos_plus_tactile_num_ = 0;  // 位置控制 + 指尖力控制时，命令的最小长度

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tactiles_publisher_;
  // std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> realtime_tactiles_publisher_;

  // DexhandController specific parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace dexhand_controller

#endif  // DEXHAND_CONTROLLER__DEXHAND_CONTROLLER_HPP_
