// Copyright 2026 botayrd.ai Robotics
//

#include "dexhand_joint_controller/dexhand_joint_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dexhand_joint_controller
{
DexhandJointController::DexhandJointController()
: ForwardControllersBase()
{
}

void DexhandJointController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn DexhandJointController::read_parameters()
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    for (const auto & interface : params_.interface_names) {
      if (interface != hardware_interface::HW_IF_POSITION &&
          interface != hardware_interface::HW_IF_VELOCITY &&
          interface != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR(get_node()->get_logger(),
                "Invalid interface name '%s' in 'interface_names' parameter. Valid options are: '%s', '%s', '%s'",
                interface.c_str(),
                hardware_interface::HW_IF_POSITION,
                hardware_interface::HW_IF_VELOCITY,
                hardware_interface::HW_IF_EFFORT);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  for (const auto & joint : params_.joints) {
    for (const auto & interface : params_.interface_names) {
      command_interface_types_.push_back(joint + "/" + interface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandJointController::on_init()
{
  auto ret = ForwardControllersBase::on_init();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandJointController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardControllersBase::on_deactivate(previous_state);

  // stop all joints
  for (auto & command_interface : command_interfaces_) {
    /*
    bool ok = command_interface.set_value(0.0);
    if (!ok)  {
      RCLCPP_ERROR(get_node()->get_logger(), "on_deactivate(): Failed to set command interface value to 0.0");
    }
    */
    command_interface.set_value(0.0);
  }

  return ret;
}

}  // namespace dexhand_joint_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dexhand_joint_controller::DexhandJointController, controller_interface::ControllerInterface)
