/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_A1_SIM_H
#define QR_ROBOT_A1_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "geometry_msgs/WrenchStamped.h"
#include "robots/qr_robot.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

namespace Quadruped {

class qrRobotA1Sim : public qrRobot {
public:
  /**
   * @brief qrRobotA1Sim 类的构造函数
   * @param nhIn: ROS 节点句柄。
   * @param privateNhIn: 私有 ROS 节点句柄。
   * @param configFilePath: 配置文件路径。
   */
  qrRobotA1Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath);

  ~qrRobotA1Sim() = default;

  /**
   * @brief 向 Gazebo 发送命令。
   * @param motorcmd: 要发送的命令。
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
   * @brief ROS 节点句柄。
   */
  ros::NodeHandle &nh;

  /**
   * @brief 私有 ROS 节点句柄。
   */
  ros::NodeHandle &privateNh;

  /**
   * @brief Unitree 低级命令，存储电机命令。
   */
  unitree_legged_msgs::LowCmd lowCmd;

  /**
   * @brief Unitree 低级状态，存储 IMU 和关节状态。
   */
  unitree_legged_msgs::LowState lowState;

  /**
   * @brief 12个关节命令发布者。
   */
  ros::Publisher jointCmdPub[12];

  /**
   * @brief 12个关节状态订阅者。
   */
  ros::Subscriber jointStateSub[12];

  /**
   * @brief 4个力传感器订阅者。
   */
  ros::Subscriber footForceSub[4];

  /**
   * @brief Gazebo IMU 订阅者。
   */
  ros::Subscriber imuSub;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_A1_SIM_H
