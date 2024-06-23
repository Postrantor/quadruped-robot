/**
 * @brief []
 * @author unitree
 * @date 2024-06-23 16:06:13
 * @copyright Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
    Use of this source code is governed by the MPL-2.0 license, see LICENSE.
 */

#ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
#define _UNITREE_JOINT_CONTROL_TOOL_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>

#define posStopF (2.146E+9f)  // stop position control mode
#define velStopF (16000.0f)   // stop velocity control mode

typedef struct {
  uint8_t mode;
  double pos;
  double posStiffness;
  double vel;
  double velStiffness;
  double torque;
} ServoCmd;

/**
 * @brief 限位函数
 * @details ros2 中倒是有现成的限位函数，可以不用这个sdk中提供的
 *  eg. clamp(1.5, -1, 1) = 1
 * @return double
 */
double clamp(double&, double, double);

/**
 * @brief get current velocity
 * 依据 current_postion/last_position/last_velocity 计算速度值
 * @param current_position
 * @param last_position
 * @param last_velocity
 * @param duration
 * @return double
 */
double computeVel(double current_position, double last_position, double last_velocity, double duration);
/**
 * @brief  get torque
 * @details 同上
 * @param current_position
 * @param current_velocity
 * @return double
 */
double computeTorque(double current_position, double current_velocity, ServoCmd&);

#endif
