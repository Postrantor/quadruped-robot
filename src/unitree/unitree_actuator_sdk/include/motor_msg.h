/**
 * @file motor_msg.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MOTOR_MSG
#define MOTOR_MSG

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef int16_t q15_t;

// 定义数据包头
typedef struct {
  unsigned char start[2];  // 包头
  unsigned char motorID;  // 电机ID  0,1,2   0xBB 表示向所有电机广播（此时无返回）
  unsigned char reserved;
} COMHead;

// 发送用单个数据，数据结构
typedef union {
  int32_t L;
  uint8_t u8[4];
  uint16_t u16[2];
  uint32_t u32;
  float F;
} COMData32;

typedef struct {
  uint8_t fan_d;   // 关节上的散热风扇转速
  uint8_t Fmusic;  // 电机发声频率   /64*1000   15.625f 频率分度
  uint8_t Hmusic;  // 电机发声强度   推荐值4  声音强度 0.1 分度
  uint8_t reserved4;
  uint8_t FRGB[4];  // 足端LED
} LowHzMotorCmd;

typedef struct {
  uint8_t mode;       // 关节模式选择
  uint8_t ModifyBit;  // 电机控制参数修改位
  uint8_t ReadBit;    // 电机控制参数发送位
  uint8_t reserved;

  COMData32 Modify;  // 电机参数修改的数据

  //实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
  // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
  q15_t T;
  // 期望关节速度 （电机本身的速度） x128, 8 + 7描述
  q15_t W;
  // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）
  int32_t Pos;
  // 关节刚度系数 x2048  4+11 描述
  q15_t K_P;
  // 关节速度系数 x1024  5+10 描述
  q15_t K_W;

  uint8_t LowHzMotorCmdIndex;  // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
  uint8_t LowHzMotorCmdByte;  // 电机低频率控制命令的字节

  COMData32 Res[1];  // 通讯保留字节用于实现别的一些通讯内容
} MasterComdV3;      // 加上数据包的包头 和CRC 34字节

// 电机控制命令数据包
typedef struct {
  COMHead head;
  MasterComdV3 Mdata;
  COMData32 CRCdata;
} MasterComdDataV3;  //返回数据

typedef struct {
  uint8_t mode;     // 当前关节模式
  uint8_t ReadBit;  // 电机控制参数修改，是否成功位
  int8_t Temp;      // 电机当前平均温度
  uint8_t MError;   // 电机错误 标识

  COMData32 Read;  // 读取的当前 电机 的控制数据

  int16_t T;   // 当前实际电机输出力矩       7 + 8 描述
  int16_t W;   // 当前实际电机速度（高速）   8 + 7 描述
  float LW;    // 当前实际电机速度（低速）
  int16_t W2;  // 当前实际关节速度（高速）   8 + 7 描述
  float LW2;   // 当前实际关节速度（低速）

  int16_t Acc;     // 电机转子加速度       15+0 描述  惯量较小
  int16_t OutAcc;  // 输出轴加速度         12+3 描述  惯量较大

  int32_t Pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
  int32_t Pos2;  // 关节编码器位置(输出编码器)

  // 电机驱动板6轴传感器数据
  int16_t gyro[3];
  int16_t acc[3];

  // 力传感器的数据
  int16_t Fgyro[3];  //
  int16_t Facc[3];
  int16_t Fmag[3];
  uint8_t Ftemp;  // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

  int16_t Force16;  // 力传感器高16位数据
  int8_t Force8;    // 力传感器低8位数据

  uint8_t FError;  //  足端传感器错误标识

  int8_t Res[1];  // 通讯 保留字节
} ServoComdV3;    // 加上数据包的包头 和CRC 78字节（4+70+4）

// 电机控制命令数据包
typedef struct {
  COMHead head;
  ServoComdV3 Mdata;
  COMData32 CRCdata;
} ServoComdDataV3;  //发送数据

#ifdef __cplusplus
}
#endif

#endif
