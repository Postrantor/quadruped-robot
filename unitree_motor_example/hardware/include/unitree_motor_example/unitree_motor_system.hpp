/**
 * @brief
 * @date 2024-02-16
 * @copyright Copyright (c) 2024
 */

#ifndef UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_
#define UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_

#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "unitree_motor_example/visibility_control.h"

// FIXME(@zhiqi.jia), should be add prefix "unitree_sdk/*"
#include "common/convert.h"
#include "motor/motor_control.hpp"
#include "serial/serial_port.hpp"
// #include "sdk/unitree_sdk.h"

namespace unitree_motor_example {

using CallbackReturn = hardware_interface::CallbackReturn;

// FIXME(@zhiqi.jia), should be in class?
static const rclcpp::Logger LOGGER = rclcpp::get_logger("UnitreeMotorSystemHardware");

class UnitreeMotorSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeMotorSystemHardware);

  /**
   * @brief 这个函数将info_变量的接口(从URDF中进行读取)，转存到一个state_interfaces中，返回到ControllerManager
   * @return std::vector<hardware_interface::StateInterface> 返回一个状态接口列表
   */
  UNITREE_MOTOR_EXAMPLE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 同样，从info中读取命令接口，存入结构体返回到ControllerManager
   * @return std::vector<hardware_interface::CommandInterface>
   */
  UNITREE_MOTOR_EXAMPLE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief parse parameters and check
   * @details 应该初始化所有成员变量并处理来自`info`参数的参数。
   *   在第一行中，通常会调用父级`on_init`来处理标准值，例如名称，父类中会赋值给`info_`成员变量。
   *   这是通过使用：`hardware_interface::(Actuator|Sensor|System)Interface::on_init(info)`来完成的。
   *   类似于 controller_manager 中的 parameter_listenter
   * @param info the parent class will assign `info` to the `info_` member variable
   * @return CallbackReturn
   * 如果所有必需的参数均已设置且有效，并且一切正常，则返回`CallbackReturn::SUCCESS`，否则返回`CallbackReturn::ERROR`。
   */
  UNITREE_MOTOR_EXAMPLE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // `read()`, `write()`
  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // time interval for operating hardware
  uint32_t hw_start_interval_millisec_;
  uint32_t hw_stop_interval_millisec_;

  // state hw_interface
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  // std::vector<double> hw_state_efforts_;

  // command hw_interface
  // std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  // std::vector<double> hw_command_efforts_;

  // unitree motor
  std::vector<std::string> hw_serial_ports_name_;
  std::vector<SerialPort> hw_serial_ports_;
  std::vector<uint8_t> hw_motor_ids_;
};

}  // namespace unitree_motor_example

#endif  // UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_
