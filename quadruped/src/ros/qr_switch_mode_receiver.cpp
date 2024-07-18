/**
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "quadruped/ros/qr_switch_mode_receiver.h"

namespace Quadruped {

qrSwitchModeReceiver::qrSwitchModeReceiver() : Node("qr_switch_mode_receiver") {
  RCLCPP_INFO(this->get_logger(), "switch mode topic: %s", switchModeTopic.c_str());
  switchModeSub = this->create_subscription<std_msgs::msg::Int8>(
      switchModeTopic, 10, std::bind(&qrSwitchModeReceiver::SwitchModeCallback, this, std::placeholders::_1));
}

void qrSwitchModeReceiver::SwitchModeCallback(const std_msgs::msg::Int8::SharedPtr input) { switchMode = input->data; }

}  // Namespace Quadruped
