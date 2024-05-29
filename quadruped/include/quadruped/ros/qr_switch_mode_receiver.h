/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_SWITCH_MODE_RECEIVER_H
#define QR_SWITCH_MODE_RECEIVER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <iostream>
#include <string>

namespace Quadruped {

/**
 * @brief Receive locomotion switch command from state machine.
 */
class qrSwitchModeReceiver {
public:
  qrSwitchModeReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

  ~qrSwitchModeReceiver() = default;

  void SwitchModeCallback(const std_msgs::Int8::ConstPtr &input);

  inline int GetSwitchMode() { return switchMode; };

  ros::NodeHandle &nh;

  ros::NodeHandle &privateNh;

  ros::Subscriber switchModeSub;

  std::string switchModeTopic = "/switch_mode";

private:
  int switchMode = 2;
};

}  // Namespace Quadruped

#endif  // QR_SWITCH_MODE_RECEIVER_H
