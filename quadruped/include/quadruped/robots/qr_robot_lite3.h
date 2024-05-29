/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_LITE2_ROBOT_H
#define QR_LITE2_ROBOT_H

#include "deeprobotics_legged_sdk/parse_cmd.h"
#include "deeprobotics_legged_sdk/send_to_robot.h"
#include "qr_robot.h"

namespace Quadruped {

class qrRobotLite3 : public qrRobot {
public:
  /**
   * @brief Constructor method of class qrRobotLite3.
   * @param config_file_path: the path to config file.
   */
  qrRobotLite3(std::string configFilePath);

  ~qrRobotLite3() = default;

  /**
   * @see qrRobot::ReceiveObservation
   */
  void ReceiveObservation();

  /**
   * @see qrRobot::ApplyAction
   */
  void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode);

  /**
   * @see qrRobot::ApplyAction
   */
  void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode);

  /**
   * @see qrRobot::Step
   */
  void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode);

  /**
   * @see qrRobot::BuildDynamicModel
   */
  virtual bool BuildDynamicModel() override;

  /**
   * @brief State receiver for lite2.
   */
  ParseCMD lite2Receiver;

  /**
   * @brief Command sender for lite2.
   */
  SendToRobot lite2Sender;

  /**
   * @brief Lite2 robot state.
   */
  RobotState lowState_lite2;
};

}  // namespace Quadruped

#endif  // QR_LITE2_ROBOT_H
