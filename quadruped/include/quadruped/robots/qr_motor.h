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
 * @brief 将电机命令发送到 uitree_sdk
 * @param p double，位置，关节角度
 * @param Kp double，位置系数
 * @param d double，关节速度
 * @param Kd double，速度系数
 * @param tua double，扭矩
 */
struct qrMotorCommand {
  double p;
  double Kp;
  double d;
  double Kd;
  double tua;

  qrMotorCommand() = default;

  /**
   * @brief qrMotorCommand 的构造函数
   * @param pIn: 关节角度输入
   * @param KpIn: 位置刚度输入
   * @param dIn: 关节速度输入
   * @param KdIn: 速度刚度输入
   * @param tuaIn: 扭矩输入
   */
  qrMotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn)
      : p(pIn), Kp(KpIn), d(dIn), Kd(KdIn), tua(tuaIn) {};

  /**
   * @brief qrMotorCommand 的构造函数
   * @param cmd: p, Kp, d, Kd 和 tau 的向量
   */
  qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd);

  /**
   * @brief 将 p, Kp, d, Kd 和 tau 设置为 0
   */
  void SetZero();

  /**
   * @brief 将 p, Kp, d, Kd 和 tau 转换为向量
   * @return 向量 < p, Kp, d, Kd 和 tau >
   */
  Eigen::Matrix<float, 5, 1> convertToVector() const;

  /**
   * @brief 重载 << 运算符以输出电机命令
   */
  friend std::ostream &operator<<(std::ostream &os, qrMotorCommand &data);

  /**
   * @brief 将命令向量转换为 Eigen 矩阵
   * @param MotorCommands: 命令向量
   * @return 命令矩阵
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
