// Copyright 2021 ros2_control Development Team
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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_motor_example/unitree_motor_system.hpp"

namespace unitree_motor_example {

/**
 * @brief parse parameters and check
 * @details
 * 实现`on_init`方法。在这里，应该初始化所有成员变量并处理来自`info`参数的参数。
 * 在第一行中，通常会调用父级`on_init`来处理标准值，例如名称。
 * 这是通过使用：`hardware_interface::(Actuator|Sensor|System)Interface::on_init(info)`来完成的。
 * @param info
 * @return hardware_interface::CallbackReturn
 * 如果所有必需的参数均已设置且有效，并且一切正常，则返回`CallbackReturn::SUCCESS`，否则返回`CallbackReturn::ERROR`。
 */
hardware_interface::CallbackReturn UnitreeMotorSystemHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. @zhiqi.jia, parse parameter to class member variables
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  hw_start_sec_ = std::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code
  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  // 2. checking for consistency between two variables
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // UnitreeMotor has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("UnitreeMotorSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("UnitreeMotorSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("UnitreeMotorSystemHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 这个函数将info_变量的接口(从URDF中进行读取)
 * 转存到一个state_interfaces中，返回到ControllerManager
 * @details
 * 定义了硬件提供的接口。对于`Sensor`类型的硬件接口，没有`export_command_interfaces`方法。
 * 提醒一下，完整的接口名称具有结构`<joint_name>/<interface_type>`。
 * $ros2_control/hardware_interface/doc/writing_new_hardware_component.md
 * @return std::vector<hardware_interface::StateInterface> 返回一个状态接口列表
 */
std::vector<hardware_interface::StateInterface>
UnitreeMotorSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

/**
 * @brief 同上
 *
 * @return std::vector<hardware_interface::CommandInterface> 返回 command
 * 接口列表
 */
std::vector<hardware_interface::CommandInterface>
UnitreeMotorSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

/**
 * @brief 实现启用硬件`power`的`on_activate`方法。
 * @details (并初始化变量)
 */
hardware_interface::CallbackReturn UnitreeMotorSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
              "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 去使能硬件，其作用与`on_activate`相反。
 */
hardware_interface::CallbackReturn UnitreeMotorSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
              "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief
 * 从硬件获取状态并将其存储到`export_state_interfaces`中定义的内部变量中。
 */
hardware_interface::return_type
UnitreeMotorSystemHardware::read(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration &period) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
    // simulate unitree motor wheels's movement as a first-order system
    // update the joint status: this is a revolute joint without any limit.
    // simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
    RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!",
                hw_positions_[i], hw_velocities_[i],
                info_.joints[i].name.c_str());

    // --- your production code ---
    // @zhiqi.jia, 需要将上面的公式替换为 unitree motor 的接口
    // 是通过 pub/sub 的方式，还是直接在这里集成 unitree 的 sdk?
    // 如果是借鉴 @rosbot 项目的方式，是直接集成了对应的硬件驱动，从硬件中读取
    // $rosbot/ros/src/odrive_hardware_interface/src/odrive_hardware_interface.cpp
    // ---
    // 在 unitree sdk 中 read/write 是一起的，虽然单独分开的
    // api，...没有弄懂怎么用 serial_.send_recv(&motor_cmd_, &motor_state_);
    // 在write()中可以舍弃读取的结果，在read()中可以将上一次指令再发一次？
    // --- production code ---
  }
  // END: This part here is for exemplary purposes - Please do not copy to your

  return hardware_interface::return_type::OK;
}

/**
 * @brief
 * 该方法根据`export_command_interfaces`中定义的内部变量中存储的值来命令硬件。
 */
hardware_interface::return_type
unitree_motor_example ::UnitreeMotorSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
                "Got command %.5f for '%s'!", hw_commands_[i],
                info_.joints[i].name.c_str());
    hw_velocities_[i] = hw_commands_[i];

    // --- your production code ---
    // @zhiqi.jia, 实际硬件中，将command->写入驱动接口
    // --- production code ---
  }
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMotorSystemHardware"),
              "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your

  return hardware_interface::return_type::OK;
}

} // namespace unitree_motor_example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_motor_example::UnitreeMotorSystemHardware,
                       hardware_interface::SystemInterface)
