/**
 * @brief
 * @copyright
 */

#include <unistd.h>
#include <chrono>
#include <thread>

#include "serial/serial_port.hpp"
#include "motor/motor_control.hpp"

int main() {
  SerialPort serial("/dev/ttyUSB0");

  MotorCmd cmd;
  MotorData data;

  static int count = 0;
  while (true) {
    if (count < 10) {
      cmd.motorType = MotorType::GO_M8010_6;
      cmd.id = 0;
      cmd.mode = 1;
      cmd.K_P = 0.0;
      cmd.K_W = 0.05;
      cmd.Pos = 0.0;
      cmd.W = 6.28 * 6.33;
      cmd.T = 0.0;

      data.motorType = MotorType::GO_M8010_6;

      serial.sendRecv(&cmd, &data);

      std::cout << "count: " << count << std::endl;
      if (data.correct == true) {
        std::cout << std::endl;
        std::cout << "motor.Pos: " << data.Pos << " rad" << std::endl;
        std::cout << "motor.Temp: " << data.Temp << " ℃" << std::endl;
        std::cout << "motor.W: " << data.W << " rad/s" << std::endl;
        std::cout << "motor.T: " << data.T << " N·m" << std::endl;
        std::cout << "motor.MError: " << data.MError << std::endl;
        std::cout << std::endl;
      }

      // uint8_t *p = (uint8_t *)cmd.get_motor_send_data();
      // for (int i = 0; i < 17; i++) {
      //   printf("0X%02X ", *p++);
      // }

      count += 1;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      cmd.motorType = MotorType::GO_M8010_6;
      data.motorType = MotorType::GO_M8010_6;
      // cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
      cmd.mode = 0;
      cmd.id = 0;

      serial.sendRecv(&cmd, &data);

      if (data.correct == true) {
        std::cout << std::endl;
        std::cout << "motor.Pos: " << data.Pos << " rad" << std::endl;
        std::cout << "motor.Temp: " << data.Temp << " ℃" << std::endl;
        std::cout << "motor.W: " << data.W << " rad/s" << std::endl;
        std::cout << "motor.T: " << data.T << " N·m" << std::endl;
        std::cout << "motor.MError: " << data.MError << std::endl;
        std::cout << std::endl;
      }
      return 0;
    }
  }
  return 0;
}
