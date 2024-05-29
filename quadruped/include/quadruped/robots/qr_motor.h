/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_MOTOR_H
#define QR_MOTOR_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace Quadruped {

/**
 * @brief send motor commond to uitree_sdk
 * @param p double, position, joint angle
 * @param Kp double, position factor
 * @param d double, joint velocity
 * @param Kd double, velocity factor
 * @param tua double, torque
 */
struct qrMotorCommand {
  double p;
  double Kp;
  double d;
  double Kd;
  double tua;

  qrMotorCommand() = default;

  /**
   * @brief constructor of qrMotorCommand
   * @param pIn: joint angle input
   * @param KpIn: position stiffness input
   * @param dIn: joint velocity input
   * @param KdIn: velocity stiffness input
   * @param tuaIn:torque input
   */
  qrMotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn)
      : p(pIn), Kp(KpIn), d(dIn), Kd(KdIn), tua(tuaIn){};

  /**
   * @brief constructor of qrMotorCommand
   * @param cmd: vector of p, Kp, d, Kd and tau
   */
  qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd);

  /**
   * @brief set p, Kp, d, Kd and tau to 0
   */
  void SetZero();

  /**
   * @brief convert p, Kp, d, Kd and tau to Vector
   * @return vector of < p, Kp, d, Kd and tau >
   */
  Eigen::Matrix<float, 5, 1> convertToVector() const;

  /**
   * @brief operator reload to output the motor command
   */
  friend std::ostream &operator<<(std::ostream &os, qrMotorCommand &data);

  /**
   * @brief convert vector of commands to eigen matrix
   * @param MotorCommands: vector of cmds
   * @return command matrix
   */
  static Eigen::Matrix<float, 5, 12> convertToMatix(const std::vector<qrMotorCommand> &MotorCommands) {
    Eigen::Matrix<float, 5, 12> MotorCommandMatrix;
    int i = 0;
    for (auto &cmd : MotorCommands) {
      MotorCommandMatrix.col(i) = cmd.convertToVector();
      ++i;
    }
    return MotorCommandMatrix;
  };
};

}  // Namespace Quadruped

#endif  // QR_MOTOR_H
