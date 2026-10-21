// Copyright 2026 botayrd.ai Robotics
//

#ifndef DEXHAND_JOINT_CONTROLLER__DEXHAND_JOINT_CONTROLLER_HPP_
#define DEXHAND_JOINT_CONTROLLER__DEXHAND_JOINT_CONTROLLER_HPP_

#include <memory>

#include "forward_command_controller/forward_controllers_base.hpp"
#include "dexhand_joint_controller/dexhand_joint_controller_parameters.hpp"

namespace dexhand_joint_controller
{
/**
 * \brief Dexhand joint controller for a set of interfaces.
 *
 * This class forwards the command signal down to a set of interfaces on the specified joint.
 *
 * \param joints Names of the joints to control.
 * \param interface_names Names of the interfaces to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class DexhandJointController : public forward_command_controller::ForwardControllersBase
{
public:
  DexhandJointController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void declare_parameters() override;
  controller_interface::CallbackReturn read_parameters() override;

  using Params = dexhand_joint_controller::Params;
  using ParamListener = dexhand_joint_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};


}  // namespace dexhand_joint_controller

#endif  // DEXHAND_JOINT_CONTROLLER__DEXHAND_JOINT_CONTROLLER_HPP_
