/**
 * @brief
 * @copyright Copyright (c) 2024
 */

#ifndef _UNITREE_LEGGED_UDP_H_
#define _UNITREE_LEGGED_UDP_H_

#include <pthread.h>
#include <stdint.h>

#include "comm.h"
#include "unitree_legged_sdk/quadruped.h"

/**
  @brief udp critical configuration:
  @param 1. initiativeDisconnect: if need disconnection after connected, another ip/port can access
  after disconnection
    -  "block", will block till data come
  @param 2. recvType
    - "block+timeout", will block till data come or timeout
    - "non block", if no data will return immediately
  @param 3. setIpPort:
    - "Y", ip/port will be set later
    - "N", ip/port not specified, as a server wait for connect
*/

namespace UNITREE_LEGGED_SDK {

constexpr int UDP_CLIENT_PORT = 8080;                      // local port
constexpr int UDP_SERVER_PORT = 8007;                      // target port
constexpr char UDP_SERVER_IP_BASIC[] = "192.168.123.10";   // target IP address
constexpr char UDP_SERVER_IP_SPORT[] = "192.168.123.161";  // target IP address

typedef enum {
  nonBlock = 0x00,
  block = 0x01,
  blockTimeout = 0x02,
} RecvEnum;

// notice: user defined data(like struct) should add crc(4byte) at the end.
class UDP {
public:
  // udp use dafault length according to level
  UDP(uint8_t level, uint16_t localPort, const char* targetIP, uint16_t targetPort);
  UDP(uint16_t localPort,
      const char* targetIP,
      uint16_t targetPort,
      int sendLength,
      int recvLength,
      bool initiativeDisconnect = false,
      RecvEnum recvType = RecvEnum::nonBlock);
  UDP(uint16_t localPort,
      int sendLength,
      int recvLength,
      bool initiativeDisconnect = false,
      RecvEnum recvType = RecvEnum::nonBlock,
      bool setIpPort = false);
  ~UDP();

  // if not indicated at constructor function
  void SetIpPort(const char* targetIP, uint16_t targetPort);
  // use in RecvEnum::blockTimeout (unit: ms)
  void SetRecvTimeout(int time);
  // initiativeDisconnect = true, disconnect for another IP to connect
  void SetDisconnectTime(float callback_dt, float disconnectTime);
  // check if can access data
  void SetAccessibleTime(float callback_dt, float accessibleTime);

  int Send();
  // directly save in buffer
  int Recv();

  void InitCmdData(HighCmd& cmd);
  void InitCmdData(LowCmd& cmd);

  int SetSend(char*);
  int SetSend(HighCmd&);
  int SetSend(LowCmd&);

  void GetRecv(char*);
  void GetRecv(HighState&);
  void GetRecv(LowState&);

  UDPState udpState;
  char* targetIP;
  uint16_t targetPort;
  char* localIP;
  uint16_t localPort;
  bool accessible = false;  // can access or not

private:
  void init(uint16_t localPort, const char* targetIP = NULL, uint16_t targetPort = 0);

  int sockFd;
  bool connected;  // udp works with connect() function, rather than server mode
  int sendLength;
  int recvLength;
  int lose_recv;

  char* recvBuf;
  char* recvAvaliable;
  char* sendBuf;

  pthread_mutex_t sendMutex;
  pthread_mutex_t recvMutex;
  pthread_mutex_t udpMutex;

  bool nonblock = true;
  // use time out method or not (unit: ms)
  int blockTimeout = -1;
  bool initiativeDisconnect = false;
};

}  // namespace UNITREE_LEGGED_SDK

#endif