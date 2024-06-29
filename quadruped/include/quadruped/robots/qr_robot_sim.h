/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author GPT4-o
 * @brief
 * @date 2024-06-23 17:54:01
 * @copyright MIT License
 */

#ifndef QR_ROBOT_SIM_H
#define QR_ROBOT_SIM_H

#include "robots/qr_robot.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "unitree_msgs/msg/low_cmd.hpp"
#include "unitree_msgs/msg/low_state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

namespace Quadruped {

/**
 * @brief a1 robot class in simulation.
 */
class qrRobotSim : public qrRobot {
public:
  /**
   * @brief Constructor of class qrRobotA1Sim
   * @param nhIn: ROS node handle.
   * @param privateNhIn: private ROS node handle.
   * @param configFilePath: config file path.
   */
  qrRobotSim(rclcpp::Node::SharedPtr &nhIn, std::string robotName, std::string homeDir);

  ~qrRobotSim() = default;

  /**
   * @see qrRobot::ReceiveObservation
   */
  void ReceiveObservation() override;

  /**
   * @see qrRobot::ApplyAction
   */
  void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::ApplyAction
   */
  void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode);

  /**
   * @see qrRobot::Step
   */
  void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::BuildDynamicModel
   */
  virtual bool BuildDynamicModel();

  void SendCommand(const std::array<float, 60> motorcmd);

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void FRhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FRthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FRcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FLhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FLthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FLcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RRhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RRthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RRcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RLhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RLthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void RLcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg);
  void FRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void FLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void RRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void RLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr &nh;
  unitree_msgs::LowCmd lowCmd;
  unitree_msgs::LowState lowState;
  rclcpp::Publisher<unitree_msgs::MotorCmd>::SharedPtr jointCmdPub[12];
  rclcpp::Subscription<unitree_msgs::MotorState>::SharedPtr jointStateSub[12];
  rclcpp::Subscription<unitree_msgs::MotorState>::SharedPtr footForceSub[4];
  rclcpp::Subscription<unitree_msgs::MotorState>::SharedPtr imuSub;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_SIM_H
