#ifndef QR_ROBOT_A1_SIM_H
#define QR_ROBOT_A1_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "robots/qr_robot.h"

namespace Quadruped {

class qrRobotA1Sim : public qrRobot {
public:
  /**
   * @brief Constructor of class qrRobotA1Sim
   * @param nhIn: ROS node handle.
   * @param privateNhIn: private ROS node handle.
   * @param configFilePath: config file path.
   */
  qrRobotA1Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath);

  ~qrRobotA1Sim() = default;

  /**
   * @brief Send command to gazebo.
   * @param motorcmd: the commands to send.
   */
  void SendCommand(const std::array<float, 60> motorcmd);

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
  virtual bool BuildDynamicModel() override;

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

  /**
   * @brief ROS node handle.
   */
  ros::NodeHandle &nh;

  /**
   * @brief ROS private node handle.
   */
  ros::NodeHandle &privateNh;

  /**
   * @brief Unitree low command that stores motor commands.
   */
  unitree_legged_msgs::LowCmd lowCmd;

  /**
   * @brief Unitree low state that stores IMU and joint states.
   */
  unitree_legged_msgs::LowState lowState;

  /**
   * @brief 12 joint command publishers.
   */
  ros::Publisher jointCmdPub[12];

  /**
   * @brief 12 joint state subscribers.
   */
  ros::Subscriber jointStateSub[12];

  /**
   * @brief 4 force sensor subscribers.
   */
  ros::Subscriber footForceSub[4];

  /**
   * @brief Gazebo IMU subscribers.
   */
  ros::Subscriber imuSub;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_A1_SIM_H