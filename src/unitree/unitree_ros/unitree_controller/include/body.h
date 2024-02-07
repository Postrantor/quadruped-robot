/**
 * @file body.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"

// Servo: 0x0A, Damping: 0x00
#define PosStopF (2.146E+9f)
#define VelStopF (16000.0f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);

}  // namespace unitree_model

#endif
