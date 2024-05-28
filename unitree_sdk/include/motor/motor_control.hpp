#ifndef __UNITREEMOTOR_H
#define __UNITREEMOTOR_H

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdint.h>
#include <optional>

// motor communication protocol
// https://design.ros2.org/articles/idl_interface_definition.html
// https://design.ros2.org/articles/generated_interfaces_cpp.html
#include "motor/motor_msg.hpp"

enum class MotorType { GO_M8010_6 };

enum class MotorMode {
  BRAKE,     // 0: 刹车
  FOC,       // 1: FOC闭环
  CALIBRATE  // 2: 电机标定(发送后等待5sec,期间禁止给电机发送消息)
};

// 定义发送格式化数据
struct MotorCmd {
public:
  MotorType motorType = MotorType::GO_M8010_6;

  int32_t hex_len = 17;
  uint16_t id;    // motor id: 0~14, 15: broadcast id, no return
  uint16_t mode;  // motor mode
  float tau;      // 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
  float dq;       // 期望关节速度(电机转子转速 rad/s)         ±804.00
  float q;        // 期望关节位置(电机转子位置 rad)           ±411774
  float k_q;      // 关节刚度系数                           0~25.599
  float k_dq;     // 关节速度系数                           0~25.599

  void modify_data(MotorCmd *);
  uint8_t *get_motor_send_data();

private:
  //电机控制数据结构体，详见motor_msg.h
  ControlData_t motor_send_data;
};

std::ostream &operator<<(std::ostream &os, const MotorCmd &cmd) {
  os << "set motor command:" << std::endl
     << "\tmotor.id: " << cmd.id << "" << std::endl
     << "\tmotor.mode: " << cmd.mode << "" << std::endl
     << "\tmotor.tau: " << cmd.tau << " N·m" << std::endl
     << "\tmotor.dq: " << cmd.dq << " rad/s" << std::endl
     << "\tmotor.q: " << cmd.q << " rad" << std::endl
     << "\tmotor.k_q: " << cmd.k_q << "" << std::endl
     << "\tmotor.k_dq: " << cmd.k_dq << std::endl;
  return os;
}

// 定义接收数据
struct MotorData {
public:
  MotorType motorType = MotorType::GO_M8010_6;

  int32_t hex_len = 16;  // 接收的命令长度: 16Byte
  bool correct = false;  // 接收数据是否完整(true完整，false不完整或断联)
  uint8_t id;            // motor id: 0~14, 15: broadcast id, no return
  uint8_t mode;          // motor mode
  int32_t temp;          // temperature: -50~127°C
  int32_t error;         // error flag: 0.正常 1.过热 2.过流 3.过压 4.编码器故障
  float tau;             // 关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
  float dq;              // 关节速度(电机转子转速 rad/s) ±804.00
  float q;               // 关节位置(电机转子位置 rad) ±411774
  int32_t foot_force;    // 足端气压传感器接口 ADC原始值

  bool extract_data(MotorData *motor_r);
  uint8_t *get_motor_recv_data();

private:
  //电机接收数据结构体，详见motor_msg.h
  MotorData_t motor_recv_data;
};

std::ostream &operator<<(std::ostream &os, const MotorData &state) {
  os << ""
     << " get motor state:" << std::endl
     << "\tmotor.correct: " << state.correct << ""
     << std::endl
     //  << "motor.id: " << state.id << "" << std::endl
     //  << "motor.mode: " << state.mode << "" << std::endl
     << "\tmotor.temp: " << state.temp << " °C" << std::endl
     << "\tmotor.error: " << state.error << "" << std::endl
     << "\tmotor.tau: " << state.tau << " N·m" << std::endl
     << "\tmotor.dq: " << state.dq << " rad/s" << std::endl
     << "\tmotor.q: " << state.q << " rad" << std::endl
     << "\tmotor.foot_force: " << state.foot_force << std::endl;
  return os;
}

unsigned short set_mode(const MotorMode &mode) { return static_cast<unsigned short>(mode); };

/**
 * @brief Get the gear ratio object
 *
 * @param type
 * @return float
 */
float get_gear_ratio(const MotorType &type = MotorType::GO_M8010_6) {
  float gear_ratio = 6.33;

  switch (type) {
    case MotorType::GO_M8010_6:
      return gear_ratio;
    default:
      std::cout << "default use go_m8060-6 motor type ratio." << std::endl;
      return gear_ratio;
  }
};

/**
 * @brief Get the crc object
 *
 * @param recv
 * @return true
 * @return false
 */
bool get_crc(const MotorData &recv) { return recv.correct; };

/**
 * @brief
 * @details for ros2-control
 * @param motor_state
 * @param hw_interface
 * @return double
 */
double from_motor_state(MotorData *motor_state, const std::string &hw_interface) {
  double hw_state;

  if (hw_interface == "position") {
    hw_state = motor_state->q;
  } else if (hw_interface == "velocity") {
    hw_state = motor_state->dq / get_gear_ratio(MotorType::GO_M8010_6);
  } else if (hw_interface == "acceleration") {
    hw_state = motor_state->tau;
  } else if (hw_interface == "effort") {
    hw_state = motor_state->tau;
  } else {
    // do something
    hw_state = motor_state->error;
  }

  return hw_state;
}

/**
 * @brief
 * @details for ros2-control
 * @param hw_command
 * @param hw_interface
 * @return MotorCmd
 */
MotorCmd to_motor_cmd(const double *hw_command, const std::string &hw_interface) {
  MotorCmd motor_cmd;
  double position{0.0};
  double velocity{0.0};
  // double acceleration{0.0};
  double effort{0.0};

  if (hw_interface == "position") {
    position = *hw_command;
  } else if (hw_interface == "velocity") {
    velocity = *hw_command * get_gear_ratio(MotorType::GO_M8010_6);
  } else if (hw_interface == "acceleration") {
    effort = *hw_command;
  } else if (hw_interface == "effort") {
    effort = *hw_command;
  } else {
    // do something
    std::cout << "set hw command error." << std::endl;
  }

  motor_cmd.motorType = MotorType::GO_M8010_6;
  motor_cmd.mode = set_mode(MotorMode::FOC);
  motor_cmd.id = 0;
  motor_cmd.k_q = 0.0;
  motor_cmd.k_dq = 0.05;
  motor_cmd.q = position;
  motor_cmd.dq = velocity;
  motor_cmd.tau = effort;

  return motor_cmd;
}

#endif  // UNITREEMOTOR_H
