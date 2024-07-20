/**
 * @brief
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author GPT4-o
 * @author postrantor
 * @date 2024-06-23 17:54:01
 * @copyright MIT License
 */

#ifndef QR_ROBOT_A1_SIM_H
#define QR_ROBOT_A1_SIM_H

#include "quadruped/robots/qr_robot.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "unitree_msgs/msg/low_cmd.hpp"
#include "unitree_msgs/msg/low_state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

namespace Quadruped {

class qrRobotA1Sim : public qrRobot {
public:
  /**
   * @brief qrRobotA1Sim 类的构造函数
   * @param nh: ROS 节点句柄。
   * @param configFilePath: 配置文件路径。
   * @note 因为实例化机器人对象的参数都存储在配置文件中，就暂时没有使用基类中的构造函数
   *       而是在自己的构造函数中先去解析配置文件，获得参数值后去初始化成员函数
   */
  qrRobotA1Sim(const rclcpp::Node::SharedPtr& nh, std::string configFilePath);

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
  void ApplyAction(const Eigen::MatrixXf& motorCommands, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::ApplyAction
   * @details for real robot. use unitree sdk
   */
  void ApplyAction(const std::vector<qrMotorCommand>& motorCommands, MotorMode motorControlMode);

  /**
   * @see qrRobot::Step
   */
  void Step(const Eigen::MatrixXf& action, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::BuildDynamicModel
   */
  virtual bool BuildDynamicModel() override;

  // callback
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void qrRobotA1Sim::MotorStateCallback(int index, const unitree_msgs::msg::MotorState::SharedPtr msg) {
    lowState.motor_state[index].mode = msg->mode;
    lowState.motor_state[index].q = msg->q;
    lowState.motor_state[index].dq = msg->dq;
    lowState.motor_state[index].tau = msg->tau;
    lowState.motor_state[index].temp = msg->temp;
  }

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

  // void qrRobotA1Sim::FootCallback(int index, const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  //   lowState.foot_force_est[index].x = msg->wrench.force.x;
  //   lowState.foot_force_est[index].y = msg->wrench.force.y;
  //   lowState.foot_force_est[index].z = msg->wrench.force.z;
  //   lowState.foot_force[index] = msg->wrench.force.z;
  // }
  // void FRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  // void FLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  // void RRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  // void RLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  /**
   * @brief ROS 节点句柄。
   */
  rclcpp::Node::SharedPtr nhIn;

  /**
   * @brief Unitree 低级命令，存储电机命令。
   */
  unitree_msgs::msg::LowCmd lowCmd;

  /**
   * @brief Unitree 低级状态，存储 IMU 和关节状态。
   */
  unitree_msgs::msg::LowState lowState;

  /**
   * @brief 12个关节命令发布者。
   */
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr jointCmdPub[12];

  /**
   * @brief 12个关节状态订阅者。
   */
  rclcpp::Subscription<unitree_msgs::msg::MotorState>::SharedPtr jointStateSub[12];

  /**
   * @brief 4个力传感器订阅者。
   */
  rclcpp::Subscription<unitree_msgs::msg::MotorState>::SharedPtr footForceSub[4];

  /**
   * @brief Gazebo IMU 订阅者。
   */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_A1_SIM_H
