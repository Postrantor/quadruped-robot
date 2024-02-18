/**
 * @brief high frequency serial communication, not that common, but useful for
 * motor communication
 * @date 2024-02-18
 * @copyright Copyright (c) 2024
 */

#ifndef __SERIALPORT_H
#define __SERIALPORT_H

#include <fcntl.h>
#include <iostream>
#include <linux/serial.h>
#include <stdint.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "motor/motor_control.hpp"
#include "serial/error_class.hpp"
#include "serial/io_port.hpp"

enum class bytesize_t { fivebits, sixbits, sevenbits, eightbits };
enum class parity_t { parity_none, parity_odd, parity_even, parity_mark, parity_space };
enum class stopbits_t { stopbits_one, stopbits_two, stopbits_one_point_five };
enum class flowcontrol_t { flowcontrol_none, flowcontrol_software, flowcontrol_hardware };

class SerialPort : public IOPort {
public:
  SerialPort(
      const std::string &portName,
      size_t recvLength = 16,
      uint32_t baudrate = 4000000,
      size_t timeOutUs = 20000,
      BlockYN blockYN = BlockYN::NO,
      bytesize_t bytesize = bytesize_t::eightbits,
      parity_t parity = parity_t::parity_none,
      stopbits_t stopbits = stopbits_t::stopbits_one,
      flowcontrol_t flowcontrol = flowcontrol_t::flowcontrol_none);

  virtual ~SerialPort();

  void resetSerial(
      size_t recvLength = 16,
      uint32_t baudrate = 4000000,
      size_t timeOutUs = 20000,
      BlockYN blockYN = BlockYN::NO,
      bytesize_t bytesize = bytesize_t::eightbits,
      parity_t parity = parity_t::parity_none,
      stopbits_t stopbits = stopbits_t::stopbits_one,
      flowcontrol_t flowcontrol = flowcontrol_t::flowcontrol_none);

  size_t send(uint8_t *sendMsg, size_t sendLength);
  size_t recv(uint8_t *recvMsg, size_t recvLength);
  size_t recv(uint8_t *recvMsg);

  bool sendRecv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength);
  bool sendRecv(MotorCmd *sendMsg, MotorData *recvMsg);
  bool sendRecv(std::vector<MotorCmd> &sendVec, std::vector<MotorData> &recvVec);

  bool send_recv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength) { return sendRecv(sendMsg, recvMsg, sendLength); }
  bool send_recv(MotorCmd *sendMsg, MotorData *recvMsg) { return sendRecv(sendMsg, recvMsg); }
  bool send_recv(std::vector<MotorCmd> &sendVec, std::vector<MotorData> &recvVec) { return sendRecv(sendVec, recvVec); }

  void test();

private:
  void _open();
  void _set();
  void _close();

  size_t _nonBlockRecv(uint8_t *recvMsg, size_t readLen);
  std::string _portName;
  uint32_t _baudrate;
  bytesize_t _bytesize;
  parity_t _parity;
  stopbits_t _stopbits;
  flowcontrol_t _flowcontrol;

  bool _xonxoff;
  bool _rtscts;
  int _fd;

  fd_set _rSet;
};

#endif  // SERIALPORT_H
