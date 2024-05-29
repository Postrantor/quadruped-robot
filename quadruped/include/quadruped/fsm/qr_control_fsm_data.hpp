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
 * @brief ControlFSMData
 */
template <typename T>
struct qrControlFSMData {
  /**
   * @brief Pointer to Robot.
   */
  Quadruped::qrRobot* quadruped;

  /**
   * @brief Pointer to StateEstimatorContainer.
   */
  Quadruped::qrStateEstimatorContainer* stateEstimators;

  /**
   * @brief Pointer to GaitGenerator.
   */
  Quadruped::qrGaitGenerator* gaitGenerator;

  /**
   * @brief Pointer to DesiredStateCommand
   */
  Quadruped::qrDesiredStateCommand* desiredStateCommand;

  /**
   * @brief Pointer to User Parameters
   */
  qrUserParameters* userParameters;

  /**
   * @brief The motor commands calculated in this control loop.
   */
  std::vector<Quadruped::qrMotorCommand> legCmd;
};

template struct qrControlFSMData<float>;

#endif  // QR_CONTROL_FSM_DATA_H
