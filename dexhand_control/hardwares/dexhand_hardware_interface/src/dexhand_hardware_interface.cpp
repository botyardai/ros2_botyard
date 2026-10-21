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

#include <limits>
#include <vector>
#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dexhand_hardware_interface/dexhand_hardware_interface.hpp"

namespace dexhand_hardware_interface {

bool DexhandHardwareInterface::check_interface_types(const std::vector<hardware_interface::InterfaceInfo>& interfaces) {
    for (const auto& interface : interfaces) {
        if (!(interface.name == hardware_interface::HW_IF_VELOCITY ||
              interface.name == hardware_interface::HW_IF_POSITION ||
              interface.name == hardware_interface::HW_IF_EFFORT)) {
            RCLCPP_FATAL(logger_, "==> unsupported interfaces: '%s'", interface.name.c_str());
            return false;
        }
    }
    return true;
}

CallbackReturn DexhandHardwareInterface::on_init(const hardware_interface::HardwareInfo& info /*ros2_control的关节*/) {
    // 对父类初始化
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    joint_prefix = info_.hardware_parameters["joint_prefix"];
    RCLCPP_INFO(logger_, "\033[1;32m ==> on_init(): in <ros2_control> joints with prefix '%s' are: \033[0m",
                joint_prefix.c_str());
    // 将配置的关节信息存入configured_joints_中
    for (auto& joint : info_.joints) {
        RCLCPP_INFO(logger_, "\033[1;32m %s\033[0m", joint.name.c_str());
        std::string joint_name = joint.name.erase(0, joint_prefix.length());
        if (joint_name == "THT0" || joint_name == "FFT0" || joint_name == "MFT0" ||
            joint_name == "RFT0" || joint_name == "LFT0") {
            tactile_configured_regions_.push_back(joint_name);
            continue;
        }
        // 关节全部加入configured_joints_中
        configured_joints_.push_back(joint_name);  // 配置的关节
        // 若没有找到从动关节，则可以添加到主动关节组，这些是发送命令的重要关节，但是位置要包含从动关节的名字
        if (std::find(coupled_joint_names.begin(), coupled_joint_names.end(),
                      joint_name) == coupled_joint_names.end()) {
            // 如果不是 速度\位置\电流的任意一种，则报错
            if (!check_interface_types(joint.command_interfaces)) {
                RCLCPP_FATAL(logger_, "Joint '%s' has unsupported command interfaces", joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            if (!check_interface_types(joint.state_interfaces)) {
                RCLCPP_FATAL(logger_, "Joint '%s' has unsupported state interfaces.", joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            // 判断不同的接口类型，赋值到不同变量中
            for (const auto& state_interface : joint.state_interfaces) {
                if (state_interface.name == hardware_interface::HW_IF_POSITION) {
                    position_configured_joints_.push_back(joint_name);
                } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
                    velocity_configured_joints_.push_back(joint_name);
                } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
                    effort_configured_joints_.push_back(joint_name);
                }
            }
        }
    }
    RCLCPP_INFO(logger_, "\033[1;32m ==> on_init(): in <ros2_control> tactiles are: \033[0m");
    for (auto& tact : info_.gpios) {
        RCLCPP_INFO(logger_, "\033[1;32m %s\033[0m", tact.name.c_str());
        std::string tact_name = tact.name.erase(0, joint_prefix.length());
        tactile_configured_regions_.push_back(tact_name);  // 配置的触觉传感器
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn DexhandHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "==> on_configure()");

    std::string port_name = info_.hardware_parameters["port_name"];
    if (port_name.empty()) {
        RCLCPP_ERROR(logger_, "port_name is empty!");
        return CallbackReturn::ERROR;
    } else {
        RCLCPP_INFO(logger_, "\033[1;32m port_name is %s\033[0m", port_name.c_str());
    }

    uint32_t baudrate = static_cast<uint32_t>(std::stoul(info_.hardware_parameters["baudrate"]));
    RCLCPP_INFO(logger_, "\033[1;32m baudrate used %u\033[0m", baudrate);

    uint32_t timeout_ms = static_cast<uint32_t>(std::stoul(info_.hardware_parameters["timeout_ms"]));
    RCLCPP_INFO(logger_, "\033[1;32m timeout_ms used %u\033[0m", timeout_ms);

    uint8_t handId = static_cast<uint8_t>(std::stoi(info_.hardware_parameters["hand_id"]));
    RCLCPP_INFO(logger_, "\033[1;32m hand_id used %u\033[0m", handId);

    // 创建dexhand_driver_实例，用于跟灵巧手真机通信
    dexhand_driver_ = std::make_shared<dexhand_driver::DexhandDriver>();
    bool ok = dexhand_driver_->initSerial(port_name, handId, baudrate, timeout_ms);
    if (!ok) {
        RCLCPP_ERROR(logger_, "----------- init dexhand_driver fail -----------");
        return CallbackReturn::ERROR;
    }
    // dexhand_driver_->setDebug(true);  //  trun on if need debug

    return CallbackReturn::SUCCESS;
}

// io_thread_func
// 因为灵巧手的控制频率太低，所以我们设计了一个中转/缓存的机制，保证不阻塞ros2_control的loop
void DexhandHardwareInterface::io_thread_func() {
    rclcpp::Rate rate(10.0);  // 10Hz
    while (rt_buffers_.io_thread_running) {
        // 1. 读取命令buffer，下发到灵巧手
        auto cmd_ptr = rt_buffers_.command_buffer.readFromRT();
        if (cmd_ptr) {
            const dexhand_driver::DataWrapper& cmdData = *cmd_ptr;
            if (cmdData.tactDataMap.empty()) {
                bool ok = dexhand_driver_->ctlDofs(cmdData.dofDataMap);
                if (!ok) {
                    RCLCPP_ERROR(logger_, "----- io_thread_func() call dexhand_driver::ctlDofs() failed ------");
                }
            } else {
                bool ok = dexhand_driver_->ctlDofsAndTactiles(cmdData.dofDataMap, cmdData.tactDataMap);
                if (!ok) {
                    RCLCPP_ERROR(logger_, "--- io_thread_func() call dexhand_driver::ctlDofsAndTactiles() failed ---");
                }
            }
        }
        // 2. 读取灵巧手状态，写入状态buffer
        dexhand_driver::DataWrapper stateData;
        bool ok = dexhand_driver_->getDofsAndTactiles(dexhand_driver::DofsMaskAll, stateData.dofDataMap,
                                               dexhand_driver::TactsMaskFingerTip, stateData.tactDataMap);
        if (!ok) {
            RCLCPP_ERROR(logger_, "----------- io_thread_func() call getDofsAndTactiles() failed -----------");
        } else {
            rt_buffers_.state_buffer.writeFromNonRT(stateData);
        }
        rate.sleep();
    }
}

CallbackReturn DexhandHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    bool ok = dexhand_driver_->connect();
    if (!ok) {
        RCLCPP_ERROR(logger_, "==> on_activate(), connect() dexhand fail");
        return CallbackReturn::ERROR;
    }
    ok = dexhand_driver_->execCmdPing();
    if (!ok) {
        RCLCPP_ERROR(logger_, "==> on_activate(), ping dexhand fail -----------");
        return CallbackReturn::ERROR;
    }
    // 启动后台线程
    rt_buffers_.io_thread_running = true;
    rt_buffers_.io_thread = std::thread(&DexhandHardwareInterface::io_thread_func, this);

    RCLCPP_INFO(logger_, "Successfully activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DexhandHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    rt_buffers_.io_thread_running = false;
    if (rt_buffers_.io_thread.joinable()) rt_buffers_.io_thread.join();
    if (dexhand_driver_) {
        bool ok = dexhand_driver_->disconnect();
        if (!ok) {
            RCLCPP_ERROR(logger_, "==> on_deactivate(), disconnect() dexhand failed!");
            return CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(logger_, "==> on_deactivate(), disconnect() dexhand...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DexhandHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "==> on_shutdown(), destroy dexhand_driver_...");

    return CallbackReturn::SUCCESS;
}

bool DexhandHardwareInterface::update_joint_state(const std::string& name,
    const std::string& hw_if_type, const dexhand_driver::DofDataMap & dofDataMap) {
    auto it = dofDataMap.find(name);
    if (it == dofDataMap.end()) {
        RCLCPP_DEBUG(logger_, "----------- update_joint_state(): %s data not found -----------", name.c_str());
        return false;
    }

    auto dofData = it->second;
    try {
        if (hw_if_type == hardware_interface::HW_IF_POSITION) {
            // 需要与RM定义的关节名保持一直，故加上joint_prefix
            set_state(joint_prefix + name + "/" + hardware_interface::HW_IF_POSITION,
                      dofData->angle_rad);
            // RCLCPP_DEBUG(logger_, "read(): update joint state: %s, position: %.10f",
            //              name.c_str(), dofData->angle_rad);
        } else if (hw_if_type == hardware_interface::HW_IF_VELOCITY) {
            set_state(joint_prefix + name + "/" + hardware_interface::HW_IF_VELOCITY,
                      dofData->speed_rad);
        } else if (hw_if_type == hardware_interface::HW_IF_EFFORT) {
            set_state(joint_prefix + name + "/" + hardware_interface::HW_IF_EFFORT,
                      static_cast<double>(dofData->torque));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "==> update_joint_state(): error: %s, joint: %s/%s may not configured",
                     e.what(), joint_prefix.c_str(), name.c_str());
        return false;
    }

    return true;
}

bool DexhandHardwareInterface::update_tactile_state(const std::string& name,
    const dexhand_driver::TactDataMap & tactDataMap) {
    auto it = tactDataMap.find(name);
    if (it == tactDataMap.end()) {
        RCLCPP_DEBUG(logger_, "----------- update_tactile_state(): %s data not found -----------", name.c_str());
        return false;
    }

    auto tactData = it->second;
    try {
            set_state(joint_prefix + name + "/" + "fx", static_cast<double>(tactData->fx));
            set_state(joint_prefix + name + "/" + "fy", static_cast<double>(tactData->fy));
            set_state(joint_prefix + name + "/" + "fz", static_cast<double>(tactData->fz));
            // RCLCPP_DEBUG(logger_, "read(): update_tactile_state for: %s, fx: %d, fy: %d, fz: %d",
            //              name.c_str(), tactData->fx, tactData->fy, tactData->fz);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "==> update_tactile_state(): error: %s, tactile: %s/%s may not configured",
                     e.what(), joint_prefix.c_str(), name.c_str());
        return false;
    }

    return true;
}

return_type DexhandHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // 只从state_buffer_读取，不与硬件通信
    // 但state_buffer_是被dexhand_driver写入的，所以关节名是不带joint_prefix
    auto state_ptr = rt_buffers_.state_buffer.readFromRT();
    if (!state_ptr) return return_type::ERROR;
    const dexhand_driver::DataWrapper& stateData = *state_ptr;
    for (const auto& name : configured_joints_) {  // 可能包含了从动关节, 需要反馈其状态
        update_joint_state(name, hardware_interface::HW_IF_POSITION, stateData.dofDataMap);
    }
    for (const auto& name : velocity_configured_joints_) {
        update_joint_state(name, hardware_interface::HW_IF_VELOCITY, stateData.dofDataMap);
    }
    for (const auto& name : effort_configured_joints_) {
        update_joint_state(name, hardware_interface::HW_IF_EFFORT, stateData.dofDataMap);
    }

    if (!stateData.tactDataMap.empty()) {
        for (const auto& name : tactile_configured_regions_) {
            update_tactile_state(name, stateData.tactDataMap);
        }
    }

    return return_type::OK;
}

bool DexhandHardwareInterface::write_joint_command(const std::string& name, const std::string& hw_if_type,
    dexhand_driver::DofDataMap& dofDataMap) {
    auto it = dexhand_driver::DofToMask.find(name);
    if (it == dexhand_driver::DofToMask.end()) {
        RCLCPP_ERROR(logger_, "----------- write_joint_command(): %s not in DofToMask -----------", name.c_str());
        return false;
    }
    try {
        // interface_name都是ResourceManager定义的，所以要保持一致(含joint_prefix)
        std::string interface_name = joint_prefix + name + "/" + hw_if_type;
        double value = get_command(interface_name);
        if (std::isnan(value)) {
            value = 0.0;
        }
        auto& dofData = dofDataMap[name];
        if (!dofData) {
            dofData = std::make_shared<dexhand_driver::DofData>();
            dofData->name = name;
            dofDataMap[name] = dofData;
        }

        // ROS端默认得到的是弧度数据
        if (hw_if_type == hardware_interface::HW_IF_POSITION) {
            dofData->angle_rad = value;
            /*
            if (name == "FAJ1") {
                RCLCPP_WARN(logger_, "====> write_joint_command(): %s, position: %.10f degree",
                                                                  name.c_str(), value/3.14159*180.0);
            }
            */
        } else if (hw_if_type == hardware_interface::HW_IF_VELOCITY) {
            if (std::abs(value) < std::numeric_limits<double>::epsilon()) {
                dofData->speed_rad = 3.14;
            } else {
                dofData->speed_rad = value;
            }
            /*
            if (name == "FAJ1") {
                RCLCPP_WARN(logger_, "====> write_joint_command(): %s, velocity: %.10f degree/s",
                                                                   name.c_str(), value/3.14159*180.0);
            }
            */
        } else if (hw_if_type == hardware_interface::HW_IF_EFFORT) {
            dofData->torque = value;
            /*
            if (name == "FAJ1") {
                RCLCPP_WARN(logger_, "====> write_joint_command(): %s, torque: %.10f", name.c_str(), value);
            }
            */
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "==> write_joint_command(): error: %s", e.what());
        return false;
    }
    return true;
}

bool DexhandHardwareInterface::write_tactile_command(const std::string& name, const std::string& tact_dim,
    dexhand_driver::TactDataMap& tactDataMap) {
    auto it = dexhand_driver::TactToMask.find(name);
    if (it == dexhand_driver::TactToMask.end()) {
        RCLCPP_ERROR(logger_, "----------- write_tactile_command(): %s not in TactToMask -----------", name.c_str());
        return false;
    }
    try {
        // tactile没有定义在URDF文件中, 但依然需要保持在同一个ResourceManager下的唯一性，所以要保持一致(含joint_prefix)
        std::string interface_name = joint_prefix + name + "/" + tact_dim;
        double value = get_command(interface_name);
        if (value < 1.0) {
            // 不控制指尖力
            return true;
        }
        auto& tactData = tactDataMap[name];
        if (!tactData) {
            tactData = std::make_shared<dexhand_driver::TactData>();
            tactData->name = name;
            tactDataMap[name] = tactData;
        }

        // ROS端默认得到的是整数，单位是0.01N
        if (tact_dim == "fz") {
            tactData->fz = static_cast<int16_t>(value);
            RCLCPP_DEBUG(logger_, "write_tactile_command(): %s, fz: %d", name.c_str(), tactData->fz);
        } else {
            RCLCPP_WARN(logger_, "write_tactile_command() unknow tactile: %s", name.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "==> write_tactile_command(): error: %s", e.what());
        return false;
    }
    return true;
}

return_type DexhandHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // 只将命令写入command_buffer_，不与硬件通信
    dexhand_driver::DataWrapper cmdData;
    for (const auto& name : position_configured_joints_) {
        write_joint_command(name, hardware_interface::HW_IF_POSITION, cmdData.dofDataMap);
    }
    for (const auto& name : velocity_configured_joints_) {
        write_joint_command(name, hardware_interface::HW_IF_VELOCITY, cmdData.dofDataMap);
    }
    for (const auto& name : effort_configured_joints_) {
        write_joint_command(name, hardware_interface::HW_IF_EFFORT, cmdData.dofDataMap);
    }
    if (!tactile_configured_regions_.empty()) {
        for (const auto& name : tactile_configured_regions_) {
            // 指尖力控制(目前只支持法向力fz)
            write_tactile_command(name, "fz", cmdData.tactDataMap);
        }
    }

    // command_buffer是被dexhand_driver读取，所以关节名要去掉joint_prefix
    rt_buffers_.command_buffer.writeFromNonRT(cmdData);
    return return_type::OK;
}

}  // namespace dexhand_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dexhand_hardware_interface::DexhandHardwareInterface, hardware_interface::SystemInterface)
