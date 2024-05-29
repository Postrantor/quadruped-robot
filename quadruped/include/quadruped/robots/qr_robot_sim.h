/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_SIM_H
#define QR_ROBOT_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "geometry_msgs/WrenchStamped.h"
#include "robots/qr_robot.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

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
  qrRobotSim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string robotName, std::string homeDir);

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
  void ImuCallback(const sensor_msgs::Imu &msg);
  void FRhipCallback(const unitree_legged_msgs::MotorState &msg);
  void FRthighCallback(const unitree_legged_msgs::MotorState &msg);
  void FRcalfCallback(const unitree_legged_msgs::MotorState &msg);
  void FLhipCallback(const unitree_legged_msgs::MotorState &msg);
  void FLthighCallback(const unitree_legged_msgs::MotorState &msg);
  void FLcalfCallback(const unitree_legged_msgs::MotorState &msg);
  void RRhipCallback(const unitree_legged_msgs::MotorState &msg);
  void RRthighCallback(const unitree_legged_msgs::MotorState &msg);
  void RRcalfCallback(const unitree_legged_msgs::MotorState &msg);
  void RLhipCallback(const unitree_legged_msgs::MotorState &msg);
  void RLthighCallback(const unitree_legged_msgs::MotorState &msg);
  void RLcalfCallback(const unitree_legged_msgs::MotorState &msg);
  void FRfootCallback(const geometry_msgs::WrenchStamped &msg);
  void FLfootCallback(const geometry_msgs::WrenchStamped &msg);
  void RRfootCallback(const geometry_msgs::WrenchStamped &msg);
  void RLfootCallback(const geometry_msgs::WrenchStamped &msg);

  ros::NodeHandle &nh;
  ros::NodeHandle &privateNh;
  unitree_legged_msgs::LowCmd lowCmd;
  unitree_legged_msgs::LowState lowState;
  ros::Publisher jointCmdPub[12];
  ros::Subscriber jointStateSub[12];
  ros::Subscriber footForceSub[4];
  ros::Subscriber imuSub;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_SIM_H
