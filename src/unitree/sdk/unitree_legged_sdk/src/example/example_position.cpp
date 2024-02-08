/**
 * @file example_position.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-02-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom(uint8_t level) : safe(LeggedType::Go1), udp(level, 8090, "192.168.123.10", 8007) {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float qDes[3] = {0};
  float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt = 0.002;  // 0.001~0.01
};

void Custom::UDPRecv() { udp.Recv(); }

void Custom::UDPSend() { udp.Send(); }

double jointLinearInterpolation(double initPos, double targetPos, double rate) {
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::RobotControl() {
  motiontime++;
  udp.GetRecv(state);
  printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  // if( motiontime >= 100){
  if (motiontime >= 0) {
    // first, get record initial position
    // if( motiontime >= 100 && motiontime < 500){
    if (motiontime >= 0 && motiontime < 10) {
      qInit[0] = state.motorState[FR_0].q;
      qInit[1] = state.motorState[FR_1].q;
      qInit[2] = state.motorState[FR_2].q;
    }
    // second, move to the origin point of a sine movement with Kp Kd
    // if( motiontime >= 500 && motiontime < 1500){
    if (motiontime >= 10 && motiontime < 400) {
      rate_count++;
      double rate = rate_count / 200.0;  // needs count to 200
      Kp[0] = 5.0;
      Kp[1] = 5.0;
      Kp[2] = 5.0;
      Kd[0] = 1.0;
      Kd[1] = 1.0;
      Kd[2] = 1.0;
      // Kp[0] = 20.0; Kp[1] = 20.0; Kp[2] = 20.0;
      // Kd[0] = 2.0; Kd[1] = 2.0; Kd[2] = 2.0;

      qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
      qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
      qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
    }
    double sin_joint1, sin_joint2;
    if (motiontime >= 400) {
      sin_count++;
      sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
      sin_joint2 = -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
      qDes[0] = sin_mid_q[0];
      qDes[1] = sin_mid_q[1];
      qDes[2] = sin_mid_q[2] + sin_joint2;
    }

    cmd.motorCmd[FR_0].q = qDes[0];
    cmd.motorCmd[FR_0].dq = 0;
    cmd.motorCmd[FR_0].Kp = Kp[0];
    cmd.motorCmd[FR_0].Kd = Kd[0];
    cmd.motorCmd[FR_0].tau = -0.65f;

    cmd.motorCmd[FR_1].q = qDes[1];
    cmd.motorCmd[FR_1].dq = 0;
    cmd.motorCmd[FR_1].Kp = Kp[1];
    cmd.motorCmd[FR_1].Kd = Kd[1];
    cmd.motorCmd[FR_1].tau = 0.0f;

    cmd.motorCmd[FR_2].q = qDes[2];
    cmd.motorCmd[FR_2].dq = 0;
    cmd.motorCmd[FR_2].Kp = Kp[2];
    cmd.motorCmd[FR_2].Kd = Kd[2];
    cmd.motorCmd[FR_2].tau = 0.0f;
  }

  if (motiontime > 10) {
    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0) exit(-1);
  }

  udp.SetSend(cmd);
}

int main(void) {
  std::cout << "communication level is set to low-level." << std::endl
            << "warning: make sure the robot is hung up." << std::endl
            << "note: the robot also needs to be set to low-level mode, otherwise it will make "
               "strange noises and this example will not run successfully! "
            << std::endl
            << "press enter to continue..." << std::endl;
  std::cin.ignore();

  // InitEnvironment();
  Custom custom(LOWLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1) {
    sleep(10);
  };

  return 0;
}
