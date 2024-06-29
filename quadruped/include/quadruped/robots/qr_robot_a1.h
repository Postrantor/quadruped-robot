/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_A1_H
#define QR_A1_ROBOT_H

#include "qr_robot.h"

namespace Quadruped {

class qrRobotA1 : public qrRobot {
public:
  /**
   * @brief Constructor method of class qrRobotA1.
   * @param config_file_path: the path to config file.
   */
  qrRobotA1(std::string config_file_path);

  ~qrRobotA1() = default;

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
  void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::Step
   */
  void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

  /**
   * @see qrRobot::BuildDynamicModel
   */
  virtual bool BuildDynamicModel() override;

  /**
   * @brief The interface to communicate with A1 robot.
   */
  // FIXME(@zhiqi.jia) :: from unitree_sdk
  RobotInterface robotInterface;
};

}  // Namespace Quadruped

#endif  // QR_A1_ROBOT_H
