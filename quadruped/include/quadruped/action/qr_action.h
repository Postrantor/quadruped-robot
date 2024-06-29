/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
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
 * @brief Quadruped命名空间用于ascend-quadruped-cpp的子目录。\n
 */
namespace Quadruped {

namespace Action {

void ShinkLeg(qrRobot *robot, float totalTime, float timeStep);

/**
 * @brief 站立动作函数。
 * @param robot 执行该动作的机器人。
 * @param standUptime 站立动作需要花费的时间。
 * @param totalTime 机器人开始执行其他操作的时间。
 * @param timeStep 每步需要花费的时间。
 */
void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep);

/**
 * @brief 坐下动作函数。
 * @param robot 执行该动作的机器人。
 * @param sitDownTime 坐下动作需要花费的时间。
 * @param timeStep 每步需要花费的时间。
 */
void SitDown(qrRobot *robot, float sitDownTime, float timeStep);

/**
 * @brief 保持站立动作函数。
 * @param robot 执行该动作的机器人。
 * @param KeepStandTime 机器人保持站立的时间。
 * @param timeStep 每步需要花费的时间。
 */
void KeepStand(qrRobot *robot, float KeepStandTime = 1.0, float timeStep = 0.001);

/**
 * @brief 控制脚动作函数。这用于步行步态。
 * @param robot 执行该动作的机器人。
 * @param locomotionController 机器人的运动控制器。
 * @param walkTime 控制脚动作需要花费的时间。
 * @param timeStep 每步需要花费的时间。
 */
void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep);

}  // Namespace Action

}  // Namespace Quadruped

#endif  // QR_ACTION_H
