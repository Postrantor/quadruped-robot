/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "ros/qr_switch_mode_receiver.h"

namespace Quadruped {

qrSwitchModeReceiver::qrSwitchModeReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn)
    : nh(nhIn), privateNh(privateNhIn) {
  ROS_INFO("switch mode topic: %s", switchModeTopic.c_str());
  switchModeSub = nh.subscribe(switchModeTopic, 10, &qrSwitchModeReceiver::SwitchModeCallback, this);
}

void qrSwitchModeReceiver::SwitchModeCallback(const std_msgs::Int8::ConstPtr &input) { switchMode = input->data; }

}  // Namespace Quadruped
