/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <array>
#include <math.h>

#include "quadruped/robots/qr_interface.h"

using namespace UNITREE_LEGGED_SDK;

LowState RobotInterface::ReceiveObservation() {
  udp.Recv();
  udp.GetRecv(state);
  return state;
}

void RobotInterface::SendCommand(std::array<float, 60> motor_cmd) {
  cmd.levelFlag = 0xff;
  for (int motor_id = 0; motor_id < 12; motor_id++) {
    cmd.motorCmd[motor_id].mode = 0x0A;
    cmd.motorCmd[motor_id].q = motor_cmd[motor_id * 5];
    cmd.motorCmd[motor_id].Kp = motor_cmd[motor_id * 5 + 1];
    cmd.motorCmd[motor_id].dq = motor_cmd[motor_id * 5 + 2];
    cmd.motorCmd[motor_id].Kd = motor_cmd[motor_id * 5 + 3];
    cmd.motorCmd[motor_id].tau = motor_cmd[motor_id * 5 + 4];
  }
  safe.PositionLimit(cmd);
  udp.SetSend(cmd);
  udp.Send();
}
