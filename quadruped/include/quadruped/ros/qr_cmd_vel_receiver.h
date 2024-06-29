/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CMDVELRECEIVER_H
#define QR_CMDVELRECEIVER_H

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace Quadruped {

/**
 * @brief Receive command velocity from high level planning module.
 */
class qrCmdVelReceiver : public rclcpp::Node {
public:
  qrCmdVelReceiver()
      : Node("qr_cmd_vel_receiver"),
        linearVel(Eigen::Matrix<float, 3, 1>::Zero()),
        angularVel(Eigen::Matrix<float, 3, 1>::Zero()) {
    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic, 10, std::bind(&qrCmdVelReceiver::CmdVelCallback, this, std::placeholders::_1));
  }

  ~qrCmdVelReceiver() = default;

  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input) {
    cmdVel = *input;
    linearVel[0] = input->linear.x;
    linearVel[1] = input->linear.y;
    linearVel[2] = input->linear.z;
    angularVel[0] = input->angular.x;
    angularVel[1] = input->angular.y;
    angularVel[2] = input->angular.z;
  }

  inline Eigen::Matrix<float, 3, 1> GetLinearVelocity() { return linearVel; }
  inline float GetAngularVelocity() { return angularVel[2]; }

  geometry_msgs::msg::Twist cmdVel;
  std::string cmdVelTopic = "/cmd_vel";

private:
  Eigen::Matrix<float, 3, 1> linearVel;
  Eigen::Matrix<float, 3, 1> angularVel;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
};

}  // Namespace Quadruped

#endif  // QR_CMDVELRECEIVER_H
