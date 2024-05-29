/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "ros/qr_cmd_vel_receiver.h"

namespace Quadruped {

qrCmdVelReceiver::qrCmdVelReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn)
    : nh(nhIn), privateNh(privateNhIn) {
  ROS_INFO("command velocity topic: %s", cmdVelTopic.c_str());
  cmdVelSub = nh.subscribe(cmdVelTopic, 10, &qrCmdVelReceiver::CmdVelCallback, this);
}

void qrCmdVelReceiver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input) {
  linearVel << input->linear.x, input->linear.y, input->linear.z;
  angularVel << input->angular.x, input->angular.y, input->angular.z;
  ROS_INFO_STREAM(
      "[CmdVelReceiver] received velocity command:" << " linear=" << input->linear.x
                                                    << " angular=" << input->angular.z);
}

}  // Namespace Quadruped
