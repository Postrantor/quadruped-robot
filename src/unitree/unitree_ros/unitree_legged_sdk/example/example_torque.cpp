/**
 * @file example_torque.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>
#include <iostream>
#include <unistd.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom(uint8_t level) : safe(LeggedType::Go1), udp(level, 8090, "192.168.123.10", 8007) {
    udp.InitCmdData(cmd);
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motion_time = 0;
  float dt = 0.002;  // 0.001~0.01
};

void Custom::UDPRecv() { udp.Recv(); }

void Custom::UDPSend() { udp.Send(); }

void Custom::RobotControl() {
  motion_time++;
  udp.GetRecv(state);
  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if (motion_time >= 500) {
    float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
    if (torque > 5.0f) torque = 5.0f;
    if (torque < -5.0f) torque = -5.0f;

    cmd.motorCmd[FR_1].q = PosStopF;
    cmd.motorCmd[FR_1].dq = VelStopF;
    cmd.motorCmd[FR_1].Kp = 0;
    cmd.motorCmd[FR_1].Kd = 0;
    cmd.motorCmd[FR_1].tau = torque;
  }
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0) exit(-1);

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
