#ifndef MOTOR_CTRL
#define MOTOR_CTRL

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_msg.h"  //The data structure of motor communication messages
#include <stdint.h>

// Define the message to be send.
typedef struct {
  // The data to be sent to motor. Details are shown in motor_msg.h
  // [电机控制数据结构体，详见motor_msg.h]
  MasterComdDataV3 motor_send_data;

  // The Bytes count of the message to be sent, it should be 34 for this motor
  // [发送命令字节数, 本电机应为34]
  int hex_len;

  // The time that message was sent
  // 发送该命令的时间, 微秒(us)
  long long send_time;

  // The values of motor commands
  // [待发送的各项数据]
  // ?

  // Motor ID
  // [电机ID]
  unsigned short id;

  // The control mode, 0:free, 5:Open loop slow turning, 10:close loop control
  // [电机控制模式，0:空闲, 5:开环缓慢转动, 10:闭环控制]
  // The following parameters are just motor's parameters, do not concern the reducer. The real
  // torque command to control board is:
  // [以下参数均为电机本身参数，与减速器无关。实际传递给控制板的指令力矩为：]
  // K_P*delta_Pos + K_W*delta_W + T
  unsigned short mode;

  // Desired output torque of motor
  // [期望电机本身的输出力矩（Nm）]
  float T;

  // Desired output speed of motor
  // [期望电机本身的速度(rad/s)]
  float W;

  // Desired shaft position of motor
  // [期望电机本身的位置（rad）]
  float Pos;

  // The position stiffness
  // [电机本身的位置刚度系数]
  float K_P;

  // The speed stiffness
  // [电机本身的速度刚度系数]
  float K_W;
} MOTOR_send;

// Define the data structure of received message.
// [定义接收数据结构]
typedef struct {
  // The data received from motor. Details are shown in motor_msg.h
  // [电机接收数据结构体，详见motor_msg.h]
  ServoComdDataV3 motor_recv_data;

  // The Bytes count of the received message, it should be 78 for this motor
  // [接收信息的字节数, 本电机应为78]

  int hex_len;
  // The time of receiving this message, microsecond(us)
  // [接收该命令的时间, 微秒(us)]

  long long resv_time;
  // Whether the received data is correct(1:correct, 0:wrong)
  // [接收数据是否完整（1完整，0不完整）]

  //解读得出的电机数据
  int correct;

  // Motor ID
  // [电机ID]
  unsigned char motor_id;

  // The control mode, 0:free, 5:Open loop slow turning, 10:close loop control
  // [电机控制模式，0:空闲, 5:开环缓慢转动, 10:闭环控制]
  unsigned char mode;

  // Temperature
  // [温度]
  int Temp;

  // Error code
  // [错误码]
  unsigned char MError;

  // The output torque of motor
  // [当前实际电机输出力矩]
  float T;

  // The motor shaft speed(without filter)
  // [前实际电机速度（无滤波器）]
  float W;

  // The motor shaft speed(with filter)
  // [当前实际电机速度（有滤波器）]
  float LW;

  // The acceleration of motor shaft
  // [电机转子加速度]
  int Acc;

  // The motor shaft position(control board zero fixed)
  // [当前电机位置（主控0点修正，电机关节还是以编码器0点为准）]
  float Pos;

  // The data of 6 axis inertial sensor on the control board
  // [电机驱动板6轴传感器数据]
  float gyro[3];
  float acc[3];
} MOTOR_recv;

// Get the current system time, microsecond(us)
// [获取当前系统时间（微秒us）]
extern long long getSystemTime();

// Compile the data to the data structure of motor
// [将数据处理为stm32需求的格式]
extern int modify_data(MOTOR_send*);

// Extract the parameter values from received data
// [将接收到的数据解读]
extern int extract_data(MOTOR_recv*);

// Calculate the CRC. Inputs are: pointer to the data to be calculated, Bytes count/4,(ignore
// remainder) [计算CRC校验码，输入为：待校验数据指针，数据包含的4字节整形数量（余数舍去）]
uint32_t crc32_core(uint32_t*, uint32_t);

#ifdef __cplusplus
}
#endif
#endif
