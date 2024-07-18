/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_SAFETY_CHECKER_H
#define QR_SAFETY_CHECKER_H

#include <iostream>

#include "quadruped/fsm/qr_control_fsm_data.hpp"

/**
 * @brief 安全检查器处理ControlFSM请求的检查。
 */
template <typename T>
class qrSafetyChecker {
public:
  /**
   * @brief qrSafetyChecker成员的构造函数
   * @param dataIn: 需要检查的数据，一般是机器人状态和混合命令。
   */
  qrSafetyChecker(qrControlFSMData<T>* dataIn) : data(dataIn) {}

  /**
   * @brief 检查机器人的方向是否安全以便控制。
   * 这是一个预控制检查。
   * @return 是否安全。
   */
  bool CheckSafeOrientation();

  /**
   * @brief 检查脚部位置不超过太远。
   * 这是一个后控制检查。
   * 目前该方法未使用。只是返回true。
   * @return 是否安全。
   */
  bool CheckPDesFoot();

  /**
   * @brief 检查力是否太大。
   * 这是一个后控制检查。
   * @return 是否安全。
   */
  bool CheckForceFeedForward();

private:
  /**
   * @brief 指向FSM数据的指针。该数据是FSM的成员。
   */
  qrControlFSMData<T>* data;
};

#endif  // QR_SAFETY_CHECKER_H
