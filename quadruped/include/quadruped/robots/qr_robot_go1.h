/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_GO1_H
#define QR_ROBOT_GO1_H

#include "robots/qr_robot.h"

namespace Quadruped {

class qrRobotGO1 : public qrRobot {
public:
  /**
   * @brief Constructor method of class qrRobotGO1.
   * @param config_file_path: the path to config file.
   */
  qrRobotGO1(std::string configFilePath);

  virtual ~qrRobotGO1() = default;

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
   * @see qrRobot::Step
   */
  void Step(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode) override;

  virtual LowState &GetLowState();

  RobotInterface robotInterface;

  std::vector<IMU> imuDatas;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_GO1_H
