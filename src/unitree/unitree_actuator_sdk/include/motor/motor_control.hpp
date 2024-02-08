#ifndef __UNITREEMOTOR_H
#define __UNITREEMOTOR_H

#include "motor/motor_msg.hpp"

#include <stdint.h>
#include <iostream>

enum class MotorType {
  A1,  // 4.8M baudrate
  B1,  // 6.0M baudrate
  GO_M8010_6
};

enum class MotorMode { BRAKE, FOC, CALIBRATE };

struct MotorCmd {
public:
  MotorCmd() {}
  MotorType motorType;
  int hex_len;
  // 电机模式 0:刹车 1:FOC闭环 2:电机标定(发送后等待5sec,期间禁止给电机发送消息)
  // 电机控制模式，0:空闲, 5:开环缓慢转动, 10:闭环控制
  unsigned short id;
  unsigned short mode;
  float tau;
  float dq;
  float q;
  float kp;
  float kd;

  void modify_data(MotorCmd* motor_s);
  uint8_t* get_motor_send_data();

  COMData32 Res;

private:
  ControlData_t GO_M8010_6_motor_send_data;
  // MasterComdDataV3 A1B1_motor_send_data;
};

struct MotorData {
public:
  MotorData() {}
  MotorType motorType;
  int hex_len;
  unsigned char motor_id;
  unsigned char mode;
  int temp;
  int merror;
  float tau;
  float dq;
  float q;

  bool correct = false;
  bool extract_data(MotorData* motor_r);
  uint8_t* get_motor_recv_data();

  int footForce;
  float LW;
  int Acc;

  float gyro[3];
  float acc[3];

private:
  MotorData_t GO_M8010_6_motor_recv_data;
  // ServoComdDataV3 A1B1_motor_recv_data;
};

// Utility Function
int queryMotorMode(MotorType motortype, MotorMode motormode);
float queryGearRatio(MotorType motortype);
#endif  // UNITREEMOTOR_H
