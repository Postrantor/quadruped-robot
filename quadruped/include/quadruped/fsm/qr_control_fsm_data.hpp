/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CONTROL_FSM_DATA_H
#define QR_CONTROL_FSM_DATA_H

#include "controllers/qr_desired_state_command.hpp"
#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_gait.h"
#include "robots/qr_robot.h"

/**
 * @brief 控制FSM数据
 */
template <typename T>
struct qrControlFSMData {
  /**
   * @brief 机器人指针。
   */
  Quadruped::qrRobot* quadruped;

  /**
   * @brief 状态估算器容器指针。
   */
  Quadruped::qrStateEstimatorContainer* stateEstimators;

  /**
   * @brief 步态计划生成器指针。
   */
  Quadruped::qrGaitGenerator* gaitGenerator;

  /**
   * @brief 期望状态命令指针。
   */
  Quadruped::qrDesiredStateCommand* desiredStateCommand;

  /**
   * @brief 用户参数指针。
   */
  qrUserParameters* userParameters;

  /**
   * @brief 本控制循环中计算的电机命令。
   */
  std::vector<Quadruped::qrMotorCommand> legCmd;
};

template struct qrControlFSMData<float>;

#endif  // QR_CONTROL_FSM_DATA_H
