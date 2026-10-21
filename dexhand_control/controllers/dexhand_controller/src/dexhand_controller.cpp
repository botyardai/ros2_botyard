// Copyright 2026 botayrd.ai Robotics
//

#include "dexhand_controller/dexhand_controller.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dexhand_controller
{
// DexhandController class
DexhandController::DexhandController()
: controller_interface::ControllerInterface(),
rt_command_ptr_(nullptr),
tactiles_publisher_(nullptr),
command_subscriber_(nullptr) { }


void DexhandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn DexhandController::read_parameters()
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  /*
  if (params_.interface_names.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter must have 3 elements.");
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

  if (params_.tactiles.size() != 5) {
    RCLCPP_ERROR(get_node()->get_logger(), "'tactiles' parameter must have 5 elements.");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    for (const auto & interface : params_.tactiles) {
      if (interface != "THT0" && interface != "FFT0" &&
          interface != "MFT0" && interface != "RFT0" &&
          interface != "LFT0") {
        RCLCPP_ERROR(get_node()->get_logger(),
          "Invalid interface name '%s' in 'tactiles' parameter. Valid options: 'THT0', 'FFT0', 'MFT0', 'RFT0', 'LFT0'",
          interface.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }
  */
  if (params_.tactiles.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'tactiles' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }


  // cmd_step_ = params_.interface_names.size();
  cmd_step_ = 3;
  least_cmd_num_ = params_.joints.size();
  least_pos_plus_tactile_num_ = least_cmd_num_ + params_.tactiles.size();
  // least_pos_plus_tactile_num_ = least_cmd_num_ + 5;

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

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandController::on_init()
{
  try {
    declare_parameters();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
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

  command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {
      const auto cmd = *msg;

      if (!std::all_of( cmd.data.cbegin(), cmd.data.cend(),
            [](const auto & value) { return std::isfinite(value); })) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Non-finite value received. Dropping message");
        return;
      }
      rt_command_ptr_.writeFromNonRT(msg);
    });

  // Create tactile publisher
  tactiles_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/state/tactiles", rclcpp::SystemDefaultsQoS());
  // realtime_tactiles_publisher_ =
  //   std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(tactiles_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
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
  bool ok = controller_interface::get_ordered_interfaces(command_interfaces_, command_interface_names_,
                                                         std::string(""), ordered_interfaces);
  if (!ok || command_interface_names_.size() != ordered_interfaces.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DexhandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // instead reset command buffer
  // rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  // we stop all joints
  for (auto & command_interface : command_interfaces_) {
    /*
    bool ok = command_interface.set_value(0.0);
    if (!ok)  {
      RCLCPP_ERROR(get_node()->get_logger(), "on_deactivate(): Failed to set command interface value to 0.0");
    }
    */
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DexhandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto dexhand_commands = rt_command_ptr_.readFromRT();

  if (dexhand_commands && (*dexhand_commands)) {
    auto cmd_ptr = *dexhand_commands;
    if (cmd_ptr->data.size() == command_interfaces_.size()) {  // full command
      for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
        command_interfaces_[index].set_value(cmd_ptr->data[index]);
      }
    } else if (cmd_ptr->data.size() == least_pos_plus_tactile_num_ ||  // position control + tactile control
               cmd_ptr->data.size() == least_cmd_num_) {  // position control only
      int index = 0;
      int base = 0;
      for (; index < least_cmd_num_; ++index) {
        base = index * cmd_step_;
        command_interfaces_[base].set_value(cmd_ptr->data[index]);
        command_interfaces_[base + 1].set_value(default_vel_);
        command_interfaces_[base + 2].set_value(default_effort_);
      }
      base = index * cmd_step_;
      bool use_default_tactile = (cmd_ptr->data.size() == least_cmd_num_);
      for (; base < least_pos_plus_tactile_num_; ) {
        if (use_default_tactile) {
          command_interfaces_[base].set_value(default_tactile_);
        } else {
          command_interfaces_[base].set_value(cmd_ptr->data[index]);
        }
        ++base;
        ++index;
      }
    } else {  // invalid command size
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "command size (%zu) must be equal to position_only (%d) or position+tactile (%d) or full_command (%zu)",
        cmd_ptr->data.size(), least_cmd_num_, least_pos_plus_tactile_num_, command_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
  }

  // Publish tactile data
  auto tactile_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  // 按照yaml文件中定义的顺序发布指尖的三维力数据
  for (auto index = 0ul; index < state_interfaces_.size(); ++index) {
    tactile_msg->data.push_back(state_interfaces_[index].get_value());
    // RCLCPP_DEBUG(get_node()->get_logger(), "state_interface name: %s", state_interfaces_[index].get_name().c_str());
  }
  tactiles_publisher_->publish(std::move(tactile_msg));

  return controller_interface::return_type::OK;
}


}  // namespace dexhand_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dexhand_controller::DexhandController, controller_interface::ControllerInterface)
