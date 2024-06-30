/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CMDVELRECEIVER_H
#define QR_CMDVELRECEIVER_H

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <Eigen/Dense>

namespace Quadruped {

/**
 * @brief Receive command velocity from high level planning module.
 */
class qrCmdVelReceiver : public rclcpp::Node {
public:
  qrCmdVelReceiver();

  ~qrCmdVelReceiver() = default;

  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input);

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
