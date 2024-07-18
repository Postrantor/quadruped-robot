/**
 * @brief
 * @date 2024-01-28
 * @copyright Copyright (c) 2024
 */

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

class Custom {
public:
  Custom(uint8_t level) : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(level, 8090, "192.168.123.10", 8007) {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::LowCmd cmd = {0};
  UNITREE_LEGGED_SDK::LowState state = {0};
  int Tpi = 0;
  int motion_time = 0;
  float dt = 0.002;  // 0.001~0.01
};

void Custom::UDPRecv() { udp.Recv(); }

void Custom::UDPSend() { udp.Send(); }

void Custom::RobotControl() {
  motion_time++;
  udp.GetRecv(state);

  // gravity compensation
  cmd.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = -0.65f;
  cmd.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = +0.65f;
  cmd.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = -0.65f;
  cmd.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = +0.65f;

  if (motion_time >= 500) {
    float speed = 2 * sin(3 * M_PI * Tpi / 1500.0);
    cmd.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = UNITREE_LEGGED_SDK::PosStopF;
    cmd.motorCmd[UNITREE_LEGGED_SDK::FR_1].dq = speed;
    cmd.motorCmd[UNITREE_LEGGED_SDK::FR_1].Kp = 0;
    cmd.motorCmd[UNITREE_LEGGED_SDK::FR_1].Kd = 4;
    cmd.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = 0.0f;
    Tpi++;
  }

  if (motion_time > 10) {
    int res1 = safe.PowerProtect(cmd, state, 1);
    // you can uncomment it for position protection
    // int res2 = safe.positionprotect(cmd, state, 10);
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

  Custom custom(UNITREE_LEGGED_SDK::LOWLEVEL);
  UNITREE_LEGGED_SDK::LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1) {
    sleep(10);
  };

  return 0;
}
