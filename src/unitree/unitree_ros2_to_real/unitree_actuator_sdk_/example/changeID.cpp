#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#define BroadAllMotorID 0xBB
#define MotorPulsator 11

int main() {
  char serial_name[100];

  MotorCmd motor_s;
  MotorData motor_r;

  motor_s.motorType = MotorType::B1;  // set the motor type, A1 or B1
  motor_r.motorType = motor_s.motorType;

  printf("please input the name of serial port.(e.g. linux:/dev/ttyusb0, windows:\\\\.\\com3)\n");
  scanf("%s", serial_name);
  printf("the serial port is %s\n", serial_name);

  SerialPort serial(serial_name);  // set the serial port name

  motor_s.id = BroadAllMotorID;
  motor_s.mode = 10;
  serial.sendRecv(&motor_s, &motor_r);

  usleep(100000);  // sleep 0.1s

  //进入拨轮模式（修改ID）
  motor_s.mode = MotorPulsator;
  motor_s.modify_data(&motor_s);
  serial.send(motor_s.get_motor_send_data(), motor_s.hex_len);

  printf("please turn the motor.\n");
  printf("one time: id=0; two times: id=1, three times: id=2\n");
  printf("id can only be 0, 1, 2\n");
  printf("once finished, press 'a'\n");

  // int c;
  while (getchar() != (int)'a')
    ;
  printf("turn finished\n");

  //保存ID
  motor_s.mode = 0;
  motor_s.modify_data(&motor_s);
  serial.send(motor_s.get_motor_send_data(), motor_s.hex_len);

  return 0;
}
