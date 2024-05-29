/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "robots/qr_motor.h"

#include <iomanip>

namespace Quadruped {

qrMotorCommand::qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd) {
  p = cmd[0];
  Kp = cmd[1];
  d = cmd[2];
  Kd = cmd[3];
  tua = cmd[4];
}

void qrMotorCommand::SetZero() {
  p = 0;
  Kp = 0;
  d = 0;
  Kd = 0;
  tua = 0;
}

Eigen::Matrix<float, 5, 1> qrMotorCommand::convertToVector() const {
  Eigen::Matrix<float, 5, 1> HybridMotorCommandVector;
  HybridMotorCommandVector << p, Kp, d, Kd, tua;
  return HybridMotorCommandVector;
}

std::ostream &operator<<(std::ostream &os, qrMotorCommand &data) {
  os << std::setprecision(3) << data.p << " " << data.Kp << " " << data.d << " " << data.Kd << " " << data.tua;
  return os;
}

}  // Namespace Quadruped
