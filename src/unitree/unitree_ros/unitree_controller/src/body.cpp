/**
 * @file body.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "body.h"

namespace unitree_model {

ros::Publisher servo_pub[12];

unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

/**
 * @brief these parameters are only for reference.
 * @details actual patameters need to be debugged if you want to run on real robot.
 */
void paramInit() {
  for (int i = 0; i < 4; i++) {
    lowCmd.motorCmd[i * 3 + 0].mode = 0x0A;
    lowCmd.motorCmd[i * 3 + 0].Kp = 70;
    lowCmd.motorCmd[i * 3 + 0].dq = 0;
    lowCmd.motorCmd[i * 3 + 0].Kd = 3;
    lowCmd.motorCmd[i * 3 + 0].tau = 0;

    lowCmd.motorCmd[i * 3 + 1].mode = 0x0A;
    lowCmd.motorCmd[i * 3 + 1].Kp = 180;
    lowCmd.motorCmd[i * 3 + 1].dq = 0;
    lowCmd.motorCmd[i * 3 + 1].Kd = 8;
    lowCmd.motorCmd[i * 3 + 1].tau = 0;

    lowCmd.motorCmd[i * 3 + 2].mode = 0x0A;
    lowCmd.motorCmd[i * 3 + 2].Kp = 300;
    lowCmd.motorCmd[i * 3 + 2].dq = 0;
    lowCmd.motorCmd[i * 3 + 2].Kd = 15;
    lowCmd.motorCmd[i * 3 + 2].tau = 0;
  }
  for (int i = 0; i < 12; i++) {
    lowCmd.motorCmd[i].q = lowState.motorState[i].q;
  }
}

void stand() {
  double pos[12] = {0.0,  0.67, -1.3,  //
                    -0.0, 0.67, -1.3,  //
                    0.0,  0.67, -1.3,  //
                    -0.0, 0.67, -1.3};
  moveAllPosition(pos, 2 * 1000);
}

void motion_init() {
  paramInit();
  stand();
}

void sendServoCmd() {
  for (int m = 0; m < 12; m++) {
    servo_pub[m].publish(lowCmd.motorCmd[m]);
  }
  ros::spinOnce();
  usleep(1000);
}

void moveAllPosition(double* targetPos, double duration) {
  double pos[12], lastPos[12], percent;
  for (int j = 0; j < 12; j++) {
    lastPos[j] = lowState.motorState[j].q;
  }
  for (int i = 1; i <= duration; i++) {
    if (!ros::ok()) break;
    percent = (double)i / duration;
    for (int j = 0; j < 12; j++) {
      lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
    }
    sendServoCmd();
  }
}

}  // namespace unitree_model
