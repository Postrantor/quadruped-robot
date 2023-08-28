/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#ifndef UNITREE_LEGGED_SDK_CPP_INTERFACE
#define UNITREE_LEGGED_SDK_CPP_INTERFACE

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
using namespace UNITREE_LEGGED_SDK;


class RobotInterface {
public:
    RobotInterface();
    LowState ReceiveObservation();
    void SendCommand(std::array<float, 60> motorcmd);
    
    Safety safe;
    UDP udp;
    LowState state = {0};
    LowCmd cmd = {0};
};

#endif