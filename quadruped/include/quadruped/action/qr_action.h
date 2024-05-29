/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ACTION_H
#define QR_ACTION_H

#include <math.h>

#include <algorithm>

#include "controllers/qr_locomotion_controller.h"
#include "robots/qr_robot.h"

/**
 * @brief namespace Quadruped is used for subdirectory of ascend-quadruped-cpp. \n
 */
namespace Quadruped {

namespace Action {

void ShinkLeg(qrRobot *robot, float totalTime, float timeStep);

/**
 * @brief stand up action function.
 * @param robot the robot to execute this action.
 * @param standUptime the time that stand up action need to cost.
 * @param totalTime the time that robot start to execute other operation.
 * @param timeStep the time that each step cost.
 */
void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep);

/**
 * @brief sit down action function.
 * @param robot the robot to execute this action.
 * @param sitDownTime the time that sit down action need to cost.
 * @param timeStep the time that each step cost.
 */
void SitDown(qrRobot *robot, float sitDownTime, float timeStep);

/**
 * @brief keep Stand action function.
 * @param robot the robot to execute this action.
 * @param KeepStandTime the time that robot keeps standing.
 * @param timeStep the time that each step cost.
 */
void KeepStand(qrRobot *robot, float KeepStandTime = 1.0, float timeStep = 0.001);

/**
 * @brief control foot action function. This is for walk gait.
 * @param robot the robot to execute this action.
 * @param locomotionController the time that robot keeps standing.
 * @param walkTime the time that control foot action need to cost.
 * @param timeStep the time that each step cost.
 */
void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep);

}  // Namespace Action

}  // Namespace Quadruped

#endif  // QR_ACTION_H
