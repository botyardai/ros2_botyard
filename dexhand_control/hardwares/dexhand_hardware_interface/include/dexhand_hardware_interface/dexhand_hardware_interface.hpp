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

#include <atomic>
#include <map>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dexhand_hardware_interface/visibility_control.h"
#include "dexhand_hardware_interface/dexhand_hardware_interface_realtime.hpp"
#include "dexhand_driver/dexhand_driver.hpp"
#include "dexhand_interfaces/msg/tactile_grid.hpp"
#include "dexhand_interfaces/msg/hand_info.hpp"

using dexhand_driver::DexhandDriver;

using hardware_interface::return_type;

namespace dexhand_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DexhandHardwareInterface : public hardware_interface::SystemInterface {
public:
    DexhandHardwareInterface() : logger_(rclcpp::get_logger("DexhandHardwareInterface")) {}

    // 新版入口：可拿到 controller_manager 的 executor，用于挂载自建节点
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
    rclcpp::Logger logger_;
    // write()/read() 跑在 RT 循环里，限流日志需要一个常驻时钟，避免每次调用新建 Clock。
    rclcpp::Clock log_clock_{RCL_STEADY_TIME};
    std::string joint_prefix; // 多只手，则每个手的关节在URDF中会加前缀来保证唯一性
    std::string communicate_; // 通信接口类型: RS485/CANFD/EtherCAT(统一转小写存储, 不区分大小写)
    std::string gesture_gpio_name_;
    uint32_t gesture_timeout_ms_ = 5000;
    std::vector<std::string> configured_joints_; // 所有<ros2_control>tag配置的joint(需要去掉joint_prefix)
    std::vector<std::string> coupled_joint_names = {"FFJ1", "MFJ1", "RFJ1", "LFJ1"}; // 从动关节
    std::vector<std::string> velocity_configured_joints_; // 配置了velocity的关节
    std::vector<std::string> effort_configured_joints_;  // 配置了effort的关节
    std::vector<std::string> tactile_configured_regions_;  // 配置了触觉传感器的区域
    std::map<std::string, std::vector<std::string>> joint_command_interfaces_;
    std::shared_ptr<DexhandDriver> dexhand_driver_ = nullptr;
    // 高低频控制解耦
    DexhandRealtimeBuffers rt_buffers_;
    // Written by IO, or lifecycle code while IO is stopped.
    GestureFeedback gesture_feedback_;
    // Owned by the ros2_control read/write loop.
    std::uint64_t synchronized_gesture_sequence_ = 0;
    dexhand_driver::DataWrapper synchronized_gesture_command_;
    void synchronize_gesture_command(const GestureFeedback& feedback);
    void io_thread_func();

    // 100Hz 网格触觉发布：自建节点挂到 controller_manager 的 executor 上
    void publish_grid_tactiles();
    rclcpp::Executor::WeakPtr executor_;
    rclcpp::Node::SharedPtr tactile_node_;
    rclcpp::Publisher<dexhand_interfaces::msg::TactileGrid>::SharedPtr tactile_publisher_;
    rclcpp::TimerBase::SharedPtr tactile_timer_;

    // 设备身份 bridge：连接成功后把 driver 探测到的产品系列/整机版本号发布一次（transient_local，
    // 晚订阅者也能拿到），由控制器层订阅并组装成 Service 响应。自建节点挂到 controller_manager
    // 的 executor，不占 RT 控制循环。
    void publish_hand_info();
    rclcpp::Node::SharedPtr hand_info_node_;
    rclcpp::Publisher<dexhand_interfaces::msg::HandInfo>::SharedPtr hand_info_publisher_;
    std::string hand_info_topic_;  // bridge topic 名（相对名，跟随节点 namespace 自动隔离多手）

    // lifecycle 核心步骤抽取，供 on_configure/on_activate 复用
    CallbackReturn init_driver_();   // 创建 DexhandDriver 实例并按 communicate_ 初始化
    CallbackReturn start_io_();      // connect + ping + 启动 io_thread + tactile_timer
    CallbackReturn stop_io_();       // 停 tactile_timer + 停 io_thread + disconnect
};

}  // namespace dexhand_hardware_interface

#endif  // DEXHAND_HARDWARE_INTERFACE__DEXHAND_HARDWARE_INTERFACE_HPP_
