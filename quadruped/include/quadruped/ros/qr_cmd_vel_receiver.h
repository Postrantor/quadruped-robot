/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CMDVELRECEIVER_H
#define QR_CMDVELRECEIVER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace Quadruped {

/**
 * @brief Receive command velocity from high level planning module.
 */
class qrCmdVelReceiver {
public:
  qrCmdVelReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);
  ~qrCmdVelReceiver() = default;

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input);
  inline Eigen::Matrix<float, 3, 1> GetLinearVelocity() { return linearVel; }
  inline float GetAngularVelocity() { return angularVel[2]; }

  geometry_msgs::Twist cmdVel;
  ros::NodeHandle &nh;
  ros::NodeHandle &privateNh;
  ros::Subscriber cmdVelSub;
  std::string cmdVelTopic = "/cmd_vel";

private:
  Eigen::Matrix<float, 3, 1> linearVel = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> angularVel = Eigen::Matrix<float, 3, 1>::Zero();
};

}  // Namespace Quadruped

#endif  // QR_CMDVELRECEIVER_H
