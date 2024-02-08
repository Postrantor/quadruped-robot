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
      data.motorType = MotorType::GO_M8010_6;
      cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);

      cmd.id = 0;
      cmd.kp = 0.0;
      cmd.kd = 0.01;
      cmd.q = 0.0;
      cmd.dq = -6.28 * queryGearRatio(MotorType::GO_M8010_6);
      cmd.tau = 0.0;

      serial.sendRecv(&cmd, &data);

      std::cout << "count: " << count << std::endl;
      if (data.correct == true) {
        std::cout << "--- --- ---" << std::endl;
        std::cout << "q: " << data.q << " rad" << std::endl;
        std::cout << "dq: " << data.dq << " rad/s" << std::endl;
        std::cout << "tau: " << data.tau << " N.m" << std::endl;
        std::cout << "temperature: " << data.temp << " °C" << std::endl;
        std::cout << "merror: " << data.merror << std::endl;
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
      cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
      cmd.id = 0;

      serial.sendRecv(&cmd, &data);

      if (data.correct == true) {
        std::cout << "--- --- ---" << std::endl;
        std::cout << "q: " << data.q << " rad" << std::endl;
        std::cout << "dq: " << data.dq << " rad/s" << std::endl;
        std::cout << "tau: " << data.tau << " N.m" << std::endl;
        std::cout << "temperature: " << data.temp << " °C" << std::endl;
        std::cout << "merror: " << data.merror << std::endl;
        std::cout << std::endl;
      }
      return 0;
    }
  }
  return 0;
}
