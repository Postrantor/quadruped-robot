/**
 * @brief
 * @copyright
 */

#ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
#define _UNITREE_JOINT_CONTROL_TOOL_H_

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <math.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

// Servo: 0x0A, Damping: 0x00
#define PMSM (0x0A)
#define BRAKE (0x00)
// #define PosStopF (2.146E+9f)
// #define VelStopF (16000.0f)

typedef struct {
  uint8_t mode;
  double pos;
  double posStiffness;
  double vel;
  double velStiffness;
  double torque;
} ServoCmd;

// eg. clamp(1.5, -1, 1) = 1
double clamp(
    double&,  //
    double,   //
    double);
// get current velocity
double computeVel(
    double current_position,  //
    double last_position,     //
    double last_velocity,     //
    double duration);
// get torque
double computeTorque(
    double current_position,  //
    double current_velocity,  //
    ServoCmd&);

#endif
