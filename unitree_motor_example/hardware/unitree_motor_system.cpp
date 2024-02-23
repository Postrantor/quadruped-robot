/**
 * @brief
 * @date 2024-02-16
 * @copyright Copyright (c) 2024
 */

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include "rclcpp/macros.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "unitree_motor_example/unitree_motor_system.hpp"
#include "unitree_motor_example/visibility_control.h"

// FIXME(@zhiqi.jia), should be add prefix "unitree_sdk/*"
#include "common/convert.h"
#include "motor/motor_control.hpp"
#include "serial/serial_port.hpp"
// #include "sdk/unitree_sdk.h"

using namespace std::chrono_literals;

namespace unitree_motor_example {

std::vector<hardware_interface::StateInterface> UnitreeMotorSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnitreeMotorSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
  }
  return command_interfaces;
}

CallbackReturn UnitreeMotorSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 1. parse parameter to class member variables
  hw_start_interval_millisec_ = std::stoul(info_.hardware_parameters["hw_start_interval_millisec"]);
  hw_stop_interval_millisec_ = std::stoul(info_.hardware_parameters["hw_stop_interval_millisec"]);

  // 2. checking for consistency between two variables
  hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // command_interface
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL_STREAM(
          LOGGER, "Joint " << joint.name.c_str() << " has " << joint.command_interfaces.size()
                           << " command interfaces found. 1 expected.");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL_STREAM(
          LOGGER, "Joint " << joint.name.c_str() << " have " << joint.command_interfaces[0].name.c_str()
                           << " command interfaces found. " << hardware_interface::HW_IF_VELOCITY << " expected.");
      return CallbackReturn::ERROR;
    }
    // state_interface
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL_STREAM(
          LOGGER, "Joint " << joint.name.c_str() << " has " << joint.state_interfaces.size() << " state interface. 2 expected.");
      return CallbackReturn::ERROR;
    }
    // FIXME(@zhiqi.jia), 这里的顺序是如何保证的？xml中的顺序？
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL_STREAM(
          LOGGER, "Joint " << joint.name.c_str() << " have " << joint.state_interfaces[0].name.c_str()
                           << " as first state interface. " << hardware_interface::HW_IF_POSITION << " expected.");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL_STREAM(
          LOGGER, "Joint " << joint.name.c_str() << " have " << joint.state_interfaces[1].name.c_str()
                           << " as second state interface. " << hardware_interface::HW_IF_VELOCITY << " expected.");
      return CallbackReturn::ERROR;
    }
  }

  // 3. about set unitree motor
  hw_motor_ids_.resize(info_.joints.size(), std::numeric_limits<uint8_t>::quiet_NaN());
  hw_serial_ports_name_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_motor_ids_[i] = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["motor_id"]));
    hw_serial_ports_name_[i] = info_.joints[i].parameters["serial_port"];
    RCLCPP_INFO_STREAM(
        LOGGER, "" << info_.joints[i].name.c_str() << " mapped to motor " << static_cast<int>(hw_motor_ids_[i])
                   << ", use serial_port: " << hw_serial_ports_name_[i] << ".");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeMotorSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO_STREAM(LOGGER, "activating ...please wait...");

  // initialize serial ports
  for (size_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO_STREAM(LOGGER, "initialize serial port: " << hw_serial_ports_name_[i] << ", waiting...");
    hw_serial_ports_.emplace_back(hw_serial_ports_name_[i]);
    rclcpp::sleep_for(std::chrono::milliseconds(hw_start_interval_millisec_));
    RCLCPP_INFO_STREAM(LOGGER, "serial port: " << hw_serial_ports_.size());
  }

  // set some default values
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (std::isnan(hw_state_positions_[i])) {
      hw_state_positions_[i] = 0;
      hw_state_velocities_[i] = 0;
      hw_command_velocities_[i] = 0;
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeMotorSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO_STREAM(LOGGER, "deactivating ...please wait...");

  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_serial_ports_[i].resetSerial();  // use unitree sdk close serial port
    rclcpp::sleep_for(std::chrono::milliseconds(hw_stop_interval_millisec_));
    RCLCPP_INFO_STREAM(LOGGER, "close serial port `" << i << "`, waiting...");
  }

  RCLCPP_INFO_STREAM(LOGGER, "successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

/**
 * @brief 从硬件获取状态并将其存储到`export_state_interfaces`中定义的内部变量中。
 */
hardware_interface::return_type UnitreeMotorSystemHardware::read(
    const rclcpp::Time & /*time*/,  //
    const rclcpp::Duration & /*period*/) {
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // unitree motor movement update the joint status: this is a revolute joint without any limit.
    // TODO(@zhiqi.jia) 直接使用(上一次) write() 中拿到的数值?
    // 因为这里都是从一个硬件中读取/写入，频率是完全同步的，不同于需要从其他(更新频率)的硬件中读取数据，可能会造成差别
    RCLCPP_INFO_STREAM(
        LOGGER, "got position state " << hw_state_positions_[i] << " and velocity state "  //
                                      << hw_state_velocities_[i] << " for " << info_.joints[i].name.c_str() << "!");
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief sending/receiving commands to the hardware
 */
hardware_interface::return_type unitree_motor_example::UnitreeMotorSystemHardware::write(
    const rclcpp::Time & /*time*/,  //
    const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO_STREAM(LOGGER, "writing...");
  MotorCmd motor_cmd;
  MotorData motor_state;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    // write()
    motor_cmd = to_motor_cmd(&hw_command_velocities_[i], hardware_interface::HW_IF_VELOCITY);
    hw_serial_ports_[i].send_recv(&motor_cmd, &motor_state);
    // read()
    hw_state_positions_[i] = from_motor_state(&motor_state, hardware_interface::HW_IF_POSITION);
    hw_state_velocities_[i] = from_motor_state(&motor_state, hardware_interface::HW_IF_VELOCITY);
    RCLCPP_INFO_STREAM(LOGGER, "write commands " << hw_command_velocities_[i] << " to " << info_.joints[i].name.c_str() << "!");
  }

  RCLCPP_INFO_STREAM(LOGGER, "joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace unitree_motor_example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    unitree_motor_example::UnitreeMotorSystemHardware,  //
    hardware_interface::SystemInterface)
