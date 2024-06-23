/**
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "qr_cmd_vel_receiver.hpp"

namespace Quadruped {

qrCmdVelReceiver::qrCmdVelReceiver() : Node("qr_cmd_vel_receiver") {
  RCLCPP_INFO(this->get_logger(), "command velocity topic: %s", cmdVelTopic.c_str());
  cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
      cmdVelTopic, 10, std::bind(&qrCmdVelReceiver::CmdVelCallback, this, std::placeholders::_1));
}

void qrCmdVelReceiver::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input) {
  linearVel << input->linear.x, input->linear.y, input->linear.z;
  angularVel << input->angular.x, input->angular.y, input->angular.z;
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "[CmdVelReceiver] received velocity command: linear=" << input->linear.x << " angular=" << input->angular.z);
}

}  // Namespace Quadruped
