#include <unistd.h>
#include <chrono>
#include <thread>

#include "serialPort/SerialPort.h"

int main() {
  SerialPort serial("/dev/ttyUSB0");
  MotorCmd cmd;
  MotorData data;

  while (true) {
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = 0;
    cmd.mode = 1;
    cmd.K_P = 0.0;
    cmd.K_W = 0.05;
    cmd.Pos = 0.0;
    cmd.W = 6.28 * 6.33;
    cmd.T = 0.0;

    serial.sendRecv(&cmd, &data);

    if (data.correct == true) {
      std::cout << "--- --- ---" << std::endl;
      std::cout << "position: " << data.Pos << " rad" << std::endl;
      std::cout << "temperature: " << data.Temp << " °C" << std::endl;
      std::cout << "w: " << data.W << " rad/s" << std::endl;
      std::cout << "tau: " << data.T << " N.m" << std::endl;
      std::cout << "merror: " << data.MError << std::endl;
      std::cout << std::endl;
    }

    // uint8_t *p = (uint8_t *)cmd.get_motor_send_data();
    // for (int i = 0; i < 17; i++) {
    //   printf("0X%02X ", *p++);
    // }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
