/**
 * @brief
 * @date 2024-01-28
 * @copyright Copyright (c) 2024
 */

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom(uint8_t level)
      : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(level, 8090, "192.168.123.161", 8082) {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::HighCmd cmd = {0};
  UNITREE_LEGGED_SDK::HighState state = {0};
  int motion_time = 0;
  float dt = 0.002;  // 0.001~0.01
};

void Custom::UDPRecv() { udp.Recv(); }

void Custom::UDPSend() { udp.Send(); }

void Custom::RobotControl() {
  motion_time += 2;
  udp.GetRecv(state);
  // printf("%d   %f\n", motion_time, state.imu.quaternion[2]);
  // 0: idle, default stand
  // 1: forced stand
  // 2: walk continuously
  cmd.mode = 0;
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;

  if (motion_time > 0 && motion_time < 1000) {
    cmd.mode = 1;
    cmd.euler[0] = -0.3;
  }
  if (motion_time > 1000 && motion_time < 2000) {
    cmd.mode = 1;
    cmd.euler[0] = 0.3;
  }
  if (motion_time > 2000 && motion_time < 3000) {
    cmd.mode = 1;
    cmd.euler[1] = -0.2;
  }
  if (motion_time > 3000 && motion_time < 4000) {
    cmd.mode = 1;
    cmd.euler[1] = 0.2;
  }
  if (motion_time > 4000 && motion_time < 5000) {
    cmd.mode = 1;
    cmd.euler[2] = -0.2;
  }
  if (motion_time > 5000 && motion_time < 6000) {
    cmd.mode = 1;
    cmd.euler[2] = 0.2;
  }
  if (motion_time > 6000 && motion_time < 7000) {
    cmd.mode = 1;
    cmd.bodyHeight = -0.2;
  }
  if (motion_time > 7000 && motion_time < 8000) {
    cmd.mode = 1;
    cmd.bodyHeight = 0.1;
  }
  if (motion_time > 8000 && motion_time < 9000) {
    cmd.mode = 1;
    cmd.bodyHeight = 0.0;
  }
  if (motion_time > 9000 && motion_time < 11000) {
    cmd.mode = 5;
  }
  if (motion_time > 11000 && motion_time < 13000) {
    cmd.mode = 7;
  }
  if (motion_time > 13000 && motion_time < 15000) {
    cmd.mode = 6;
  }
  if (motion_time > 15000 && motion_time < 16000) {
    cmd.mode = 0;
  }
  if (motion_time > 16000 && motion_time < 20000) {
    cmd.mode = 2;
    cmd.gaitType = 2;
    cmd.velocity[0] = 0.4f;
    cmd.yawSpeed = 2;
    cmd.footRaiseHeight = 0.1;
  }
  if (motion_time > 20000 && motion_time < 22000) {
    cmd.mode = 0;
    cmd.velocity[0] = 0;
  }
  if (motion_time > 22000 && motion_time < 26000) {
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = 0.2f;
    cmd.bodyHeight = 0.1;
  }

  // straightHand mode usage
  if (motion_time > 26000 && motion_time < 27000) {
    cmd.mode = 1;
  }
  if (motion_time > 27000 && motion_time < 35000) {
    cmd.mode = 11;
  }

  // jumpYaw mode usage
  if (motion_time > 35000 && motion_time < 36000) {
    cmd.mode = 1;
  }
  if (motion_time > 36000 && motion_time < 37000) {
    cmd.mode = 10;
  }

  udp.SetSend(cmd);
}

int main(void) {
  std::cout << "communication level is set to high-level." << std::endl
            << "warning: make sure the robot is standing on the ground." << std::endl
            << "press enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);
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
