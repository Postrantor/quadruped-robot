/**
 * @brief
 * @date 2024-01-27
 * @copyright Copyright (c) 2024
 */

#ifndef _UNITREE_MSG_CONVERT_H_
#define _UNITREE_MSG_CONVERT_H_

#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/msg/bms_cmd.hpp"
#include "unitree_msgs/msg/bms_state.hpp"
#include "unitree_msgs/msg/high_cmd.hpp"
#include "unitree_msgs/msg/high_state.hpp"
#include "unitree_msgs/msg/imu.hpp"
#include "unitree_msgs/msg/low_cmd.hpp"
#include "unitree_msgs/msg/low_state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

#include "motor/motor_control.hpp"
#include "motor/motor_msg.hpp"
// #include "sdk/unitree_sdk.h"

namespace UNITREE_LEGGED_SDK {

// msg->motor
MotorCmd msg2motor(const unitree_msgs::msg::MotorCmd &msg) {
  MotorCmd cmd;

  cmd.hex_len = msg.hex_len;  // 17;
  cmd.id = msg.id;            // motor id: 0~14, 15: broadcast id, no return
  cmd.mode = msg.mode;        // motor mode
  cmd.tau = msg.tau;    // 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
  cmd.dq = msg.dq;      // 期望关节速度(电机转子转速 rad/s) ±804.00
  cmd.q = msg.q;        // 期望关节位置(电机转子位置 rad) ±411774
  cmd.k_q = msg.k_q;    // 关节刚度系数 0~25.599
  cmd.k_dq = msg.k_dq;  // 关节速度系数 0~25.599

  return cmd;
}

/**
unitree_sdk::BmsCmd msg2motor(const unitree_msgs::msg::BmsCmd &msg) {
  unitree_sdk::BmsCmd cmd;
  cmd.off = msg.off;

  for (int i(0); i < 3; i++) {
    cmd.reserve[i] = msg.reserve[i];
  }
  return cmd;
}

unitree_sdk::HighCmd msg2motor(const unitree_msgs::msg::HighCmd::SharedPtr &msg)
{ unitree_sdk::HighCmd cmd;

  for (int i(0); i < 2; i++) {
    cmd.head[i] = msg->head[i];
    cmd.SN[i] = msg->sn[i];
    cmd.version[i] = msg->version[i];
    cmd.position[i] = msg->position[i];
    cmd.velocity[i] = msg->velocity[i];
  }

  for (int i(0); i < 3; i++) {
    cmd.euler[i] = msg->euler[i];
  }

  for (int i(0); i < 4; i++) {
    cmd.led[i].r = msg->led[i].r;
    cmd.led[i].g = msg->led[i].g;
    cmd.led[i].b = msg->led[i].b;
  }

  for (int i(0); i < 40; i++) {
    cmd.wirelessRemote[i] = msg->wireless_remote[i];
  }

  cmd.levelFlag = msg->level_flag;
  cmd.frameReserve = msg->frame_reserve;
  cmd.bandWidth = msg->band_width;
  cmd.mode = msg->mode;
  cmd.gaitType = msg->gait_type;
  cmd.speedLevel = msg->speed_level;
  cmd.footRaiseHeight = msg->foot_raise_height;
  cmd.bodyHeight = msg->body_height;
  cmd.yawSpeed = msg->yaw_speed;
  cmd.reserve = msg->reserve;
  cmd.crc = msg->crc;

  cmd.bms = msg2motor(msg->bms);
  return cmd;
}

unitree_sdk::LowCmd msg2motor(const unitree_msgs::msg::LowCmd::SharedPtr &msg) {
  unitree_sdk::LowCmd cmd;

  for (int i(0); i < 2; i++) {
    cmd.head[i] = msg->head[i];
    cmd.SN[i] = msg->sn[i];
    cmd.version[i] = msg->version[i];
  }
  for (int i(0); i < 40; i++) {
    cmd.wirelessRemote[i] = msg->wireless_remote[i];
  }
  for (int i(0); i < 20; i++) {
    cmd.motorCmd[i] = msg2motor(msg->motor_cmd[i]);
  }

  cmd.bms = msg2motor(msg->bms);
  cmd.levelFlag = msg->level_flag;
  cmd.frameReserve = msg->frame_reserve;
  cmd.bandWidth = msg->band_width;
  cmd.reserve = msg->reserve;
  cmd.crc = msg->crc;
  return cmd;
}

// motor->msg
unitree_msgs::msg::IMU motor2msg(unitree_sdk::IMU &state) {
  unitree_msgs::msg::IMU msg;

  for (int i(0); i < 4; i++) {
    msg.quaternion[i] = state.quaternion[i];
  }
  for (int i(0); i < 3; i++) {
    msg.gyroscope[i] = state.gyroscope[i];
    msg.accelerometer[i] = state.accelerometer[i];
    msg.rpy[i] = state.rpy[i];
  }

  msg.temperature = state.temperature;
  return msg;
}

unitree_msgs::msg::Cartesian motor2msg(unitree_sdk::Cartesian &state) {
  unitree_msgs::msg::Cartesian msg;

  msg.x = state.x;
  msg.y = state.y;
  msg.z = state.z;

  return msg;
}

unitree_msgs::msg::BmsState motor2msg(unitree_sdk::BmsState &state) {
  unitree_msgs::msg::BmsState msg;

  for (int i(0); i < 2; i++) {
    msg.bq_ntc[i] = state.BQ_NTC[i];
    msg.mcu_ntc[i] = state.MCU_NTC[i];
  }
  for (int i(0); i < 10; i++) {
    msg.cell_vol[i] = state.cell_vol[i];
  }

  msg.version_h = state.version_h;
  msg.version_l = state.version_l;
  msg.bms_status = state.bms_status;
  msg.soc = state.SOC;
  msg.current = state.current;
  msg.cycle = state.cycle;

  return msg;
}

unitree_msgs::msg::HighState motor2msg(unitree_sdk::HighState &state) {
  unitree_msgs::msg::HighState msg;

  for (int i(0); i < 2; i++) {
    msg.head[i] = state.head[i];
    msg.sn[i] = state.SN[i];
    msg.version[i] = state.version[i];
  }
  for (int i(0); i < 4; i++) {
    msg.foot_force[i] = state.footForce[i];
    msg.foot_force_est[i] = state.footForceEst[i];
    msg.range_obstacle[i] = state.rangeObstacle[i];
    msg.foot_position2body[i] = motor2msg(state.footPosition2Body[i]);
    msg.foot_speed2body[i] = motor2msg(state.footSpeed2Body[i]);
  }
  for (int i(0); i < 3; i++) {
    msg.position[i] = state.position[i];
    msg.velocity[i] = state.velocity[i];
  }
  for (int i(0); i < 40; i++) {
    msg.wireless_remote[i] = state.wirelessRemote[i];
  }
  for (int i(0); i < 20; i++) {
    msg.motor_state[i] = motor2msg(state.motorState[i]);
  }

  msg.imu = motor2msg(state.imu);
  msg.bms = motor2msg(state.bms);

  msg.level_flag = state.levelFlag;
  msg.frame_reserve = state.frameReserve;
  msg.band_width = state.bandWidth;
  msg.mode = state.mode;
  msg.progress = state.progress;
  msg.gait_type = state.gaitType;
  msg.foot_raise_height = state.footRaiseHeight;
  msg.body_height = state.bodyHeight;
  msg.yaw_speed = state.yawSpeed;
  msg.reserve = state.reserve;
  msg.crc = state.crc;

  return msg;
}

unitree_msgs::msg::LowState motor2msg(unitree_sdk::LowState &state) {
  unitree_msgs::msg::LowState msg;

  for (int i(0); i < 2; i++) {
    msg.head[i] = state.head[i];
    msg.sn[i] = state.SN[i];
    msg.version[i] = state.version[i];
  }
  for (int i(0); i < 4; i++) {
    msg.foot_force[i] = state.footForce[i];
    msg.foot_force_est[i] = state.footForceEst[i];
  }
  for (int i(0); i < 40; i++) {
    msg.wireless_remote[i] = state.wirelessRemote[i];
  }
  for (int i(0); i < 20; i++) {
    msg.motor_state[i] = motor2msg(state.motorState[i]);
  }

  msg.imu = motor2msg(state.imu);
  msg.bms = motor2msg(state.bms);

  msg.tick = state.tick;
  msg.reserve = state.reserve;
  msg.crc = state.crc;

  return msg;
}
*/

unitree_msgs::msg::MotorState motor2msg(MotorData &state) {
  unitree_msgs::msg::MotorState msg;

  msg.hex_len = state.hex_len;  // 接收的命令长度: 16Byte
  msg.correct = state.correct;  // 接收数据是否完整(true完整，false不完整或断联)
  msg.id = state.id;            // motor id: 0~14, 15: broadcast id, no return
  msg.mode = state.mode;        // motor mode
  msg.temp = state.temp;        // temperature: -50~127°C
  msg.error = state.error;      // error flag: 0.正常 1.过热 2.过流 3.过压 4.编码器故障
  msg.tau = state.tau;          // 关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
  msg.dq = state.dq;            // 关节速度(电机转子转速 rad/s) ±804.00
  msg.q = state.q;              // 关节位置(电机转子位置 rad) ±411774
  msg.foot_force = state.foot_force;  // 足端气压传感器接口 ADC原始值

  return msg;
}

}  // namespace UNITREE_LEGGED_SDK

namespace unitree_sdk = UNITREE_LEGGED_SDK;

#endif  // _UNITREE_MSG_CONVERT_H_
