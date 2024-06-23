/**
 * @author unitree
 * @brief
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

// eg. clamp(1.5, -1, 1) = 1
double clamp(double&, double, double);
// get current velocity
double computeVel(double current_position, double last_position, double last_velocity, double duration);
// get torque
double computeTorque(double current_position, double current_velocity, ServoCmd&);

#endif
