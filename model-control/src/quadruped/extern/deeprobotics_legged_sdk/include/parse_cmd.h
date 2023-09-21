/**
 * @file robot_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PARSE_CMD_H_
#define PARSE_CMD_H_

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#include "command_list.h"
#include "robot_types.h"
#include "udpserver.hpp"
#define LOCAL_PORT 43897

#define JOINT_POS_CMD 0x0902
#define JOINT_VEL_CMD 0x0903
#define JOINT_TOR_CMD 0x0904

class ParseCMD {
private:
    RobotState state_rec;

public:
    void startWork();
    void work();
    RobotState &get_recv();
};

#endif// PARSE_CMD_H_