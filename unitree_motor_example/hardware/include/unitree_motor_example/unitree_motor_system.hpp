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

#ifndef UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_
#define UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unitree_motor_example/visibility_control.h"

namespace unitree_motor_example {
class UnitreeMotorSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeMotorSystemHardware);

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // `read()`, `write()`
  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  UNITREE_MOTOR_EXAMPLE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

private:
  // @zhiqi.jia, `info_` paser from "*.ros2_control.xacro",
  // 应该是framework已经给解析好了，这些文件已经是按照指定的方式加载进来的
  // parameters for the unitree_motor simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  // store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

} // namespace unitree_motor_example

#endif // UNITREE_MOTOR_EXAMPLE__UNITREE_MOTOR_SYSTEM_HPP_
