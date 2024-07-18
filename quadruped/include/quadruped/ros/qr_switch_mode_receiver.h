#ifndef QR_SWITCH_MODE_RECEIVER_H
#define QR_SWITCH_MODE_RECEIVER_H

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int8.hpp"

#include <iostream>
#include <string>

namespace Quadruped {

/**
 * @brief Receive locomotion switch command from state machine.
 */
class qrSwitchModeReceiver : public rclcpp::Node {
public:
  qrSwitchModeReceiver();
  ~qrSwitchModeReceiver() = default;

  void SwitchModeCallback(const std_msgs::msg::Int8::SharedPtr input);
  inline int GetSwitchMode() { return switchMode; }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr switchModeSub;
  std::string switchModeTopic = "/switch_mode";

private:
  int switchMode = 2;
};

}  // Namespace Quadruped

#endif  // QR_SWITCH_MODE_RECEIVER_H
