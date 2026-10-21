// Copyright (c) 2025,
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEXHAND_HARDWARE_INTERFACE__DEXHAND_HARDWARE_INTERFACE_HPP_
#define DEXHAND_HARDWARE_INTERFACE__DEXHAND_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dexhand_hardware_interface/visibility_control.h"
#include "dexhand_hardware_interface/dexhand_hardware_interface_realtime.hpp"
#include "dexhand_driver/dexhand_driver.hpp"

// #define DEBUG
using dexhand_driver::DexhandDriver;

using hardware_interface::return_type;

namespace dexhand_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DexhandHardwareInterface : public hardware_interface::SystemInterface {
public:
    DexhandHardwareInterface() : logger_(rclcpp::get_logger("DexhandHardwareInterface")) {}

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    bool check_interface_types(const std::vector<hardware_interface::InterfaceInfo>& interfaces);
    bool update_joint_state(const std::string& name, const std::string& hw_if_type,
        const dexhand_driver::DofDataMap& dofDataMap);
    bool write_joint_command(const std::string& name, const std::string& hw_if_type,
        dexhand_driver::DofDataMap& dofDataMap);
    bool update_tactile_state(const std::string& name, const dexhand_driver::TactDataMap& tactDataMap);
    bool write_tactile_command(const std::string& name, const std::string& tact_dim,
        dexhand_driver::TactDataMap& tactDataMap);

private:
    std::mutex mutex_;
    rclcpp::Logger logger_;
    std::string joint_prefix; // 多只手，则每个手的关节在URDF中会加前缀来保证唯一性
    std::vector<std::string> configured_joints_; // 所有<ros2_control>tag配置的joint(需要去掉joint_prefix)
    std::vector<std::string> coupled_joint_names = {"FFJ1", "MFJ1", "RFJ1", "LFJ1"}; // 从动关节
    std::vector<std::string> position_configured_joints_; // 配置了position的关节
    std::vector<std::string> velocity_configured_joints_; // 配置了velocity的关节
    std::vector<std::string> effort_configured_joints_;  // 配置了effort的关节
    std::vector<std::string> tactile_configured_regions_;  // 配置了触觉传感器的区域
    std::shared_ptr<DexhandDriver> dexhand_driver_ = nullptr;
    // 高低频控制解耦
    DexhandRealtimeBuffers rt_buffers_;
    void io_thread_func();
};

}  // namespace dexhand_hardware_interface

#endif  // DEXHAND_HARDWARE_INTERFACE__DEXHAND_HARDWARE_INTERFACE_HPP_
