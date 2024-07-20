/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_H
#define QR_ROBOT_H

#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "quadruped/config/qr_config.h"
#include "quadruped/config/qr_enum_types.h"
#include "quadruped/controllers/qr_state_dataflow.h"
#include "quadruped/dynamics/floating_base_model.hpp"
#include "quadruped/estimators/qr_moving_window_filter.hpp"
#include "quadruped/robots/qr_motor.h"
#include "quadruped/robots/qr_timer.h"
#include "quadruped/robots/qr_interface.h"
#include "quadruped/utils/qr_print.hpp"
#include "quadruped/utils/qr_se3.h"
#include "quadruped/utils/qr_tools.h"

namespace Quadruped {

class qrRobot {
public:
  /**
   * @brief qrRobot类的构造函数
   */
  qrRobot();

  /**
   * @brief qrRobot类的构造函数
   * @param robot_name: 机器人类型
   * @param config_file_path: 机器人配置文件路径
   */
  qrRobot(std::string robot_name, std::string config_file_path)
      : robotName(robot_name), configFilePath(config_file_path), timer(useRosTime){};

  virtual ~qrRobot() = default;

  /**
   * @brief 重置机器人状态
   */
  virtual void Reset();

  /**
   * @brief 在每个循环中更新观测并获取机器人状态
   * 这个方法由不同机器人自行定义
   */
  virtual void ReceiveObservation() = 0;

  /**
   * @brief 设置并发送命令给电机
   * @param motor_commands: 要执行的电机命令矩阵
   * @param motor_control_mode: 例如制动的控制模式
   */
  virtual void ApplyAction(const Eigen::MatrixXf &motor_commands, MotorMode motor_control_mode) = 0;

  /**
   * @brief 设置并发送命令给电机
   * @param motor_commands: 要执行的电机命令向量
   * @param motor_control_mode: 例如制动的控制模式
   */
  virtual void ApplyAction(const std::vector<qrMotorCommand> &motor_commands, MotorMode motor_control_mode) {}

  /**
   * @brief 执行一次观测操作并应用命令
   * @param action: 要执行的命令
   * @param motor_control_mode: 控制模式
   */
  virtual void Step(const Eigen::MatrixXf &action, MotorMode motor_control_mode) = 0;

  /**
   * @brief 执行一次观测操作并应用命令
   * @param motor_commands: 要执行的命令
   * @param motor_control_mode: 控制模式
   */
  virtual void Step(const std::vector<qrMotorCommand> &motor_commands, MotorMode motor_control_mode) {
    std::cout << "apply a step() ..." << std::endl;
  }

  /**
   * @brief 更新一些动力学数据，如速度和一些旋转矩阵
   */
  void UpdateDataFlow();

  /**
   * @brief 通过动力学公式计算力矩
   */
  void ComputeMoment();

  /**
   * @brief 将足端在臀部坐标系下的位置转换为关节角
   * @param foot_position: 足端在臀部坐标系下的位置
   * @param hip_sign: 臀部位置
   * @return 关节角
   */
  Vec3<float> FootPositionInHipFrameToJointAngle(Vec3<float> &foot_position, int hip_sign = 1);

  /**
   * @brief 获取足端在臀部坐标系下的位置
   * @param angles: 一条腿的关节角
   * @param hip_sign: 臀部位置
   * @return 足端在臀部坐标系下的位置
   */
  Vec3<float> FootPositionInHipFrame(Vec3<float> &angles, int hip_sign = 1);

  /**
   * @brief 计算一条腿的雅可比矩阵
   * @param leg_angles: 一条腿的三个关节角
   * @param legId: 要计算的腿的ID
   * @return 雅可比矩阵
   */
  Mat3<float> AnalyticalLegJacobian(Vec3<float> &leg_angles, int leg_id);

  /**
   * @brief 计算足端在机器人基坐标系下的位置
   * @param foot_angles: 关节角
   * @return 足端在基坐标系下的位置
   */
  Mat34<float> FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> foot_angles);

  /**
   * @brief 计算足端在机器人基坐标系下的速度
   * @return 足端在基坐标系下的速度
   */
  Mat34<float> ComputeFootVelocitiesInBaseFrame();

  /**
   * @brief 将足端位置转换为关节角
   * @param leg_id: 要计算的腿的ID
   * @param foot_local_position: 足端在本地坐标系下的位置
   * @param joint_idx: 要计算的关节的索引
   * @param joint_angles: 要计算的关节角
   * @return 位置(x, y, z)
   */
  void ComputeMotorAnglesFromFootLocalPosition(
      int leg_id, Vec3<float> foot_local_position, Vec3<int> &joint_idx, Vec3<float> &joint_angles);

  /**
   * @brief 计算关节速度
   * @param leg_id: 要计算的腿的ID
   * @param leg_angles: 一条腿的三个关节角
   * @param foot_local_velocity: 足端在本地坐标系下的速度
   * @return 关节速度
   */
  Vec3<float> ComputeMotorVelocityFromFootLocalVelocity(
      int leg_id, Vec3<float> leg_angles, Vec3<float> foot_local_velocity);

  /**
   * @brief 获取足端在世界坐标系下的位置
   * @param use_input: 是否使用给定的基坐标系位置和方向
   * @param base_position: 机器人基坐标系下的位置
   * @param base_orientation: 机器人基坐标系下的方向
   * @return 足端在世界坐标系下的位置
   */
  Mat34<float> GetFootPositionsInWorldFrame(
      bool use_input = false,
      Vec3<float> base_position = {0.f, 0.f, 0.f},
      Quat<float> base_orientation = {1.f, 0.f, 0.f, 0.f});

  /**
   * @brief 获取给定腿ID的雅可比矩阵
   * @param leg_id: 要计算的腿的ID
   * @return 当前的雅可比矩阵
   */
  Eigen::Matrix<float, 3, 3> ComputeJacobian(int leg_id);

  /**
   * @brief 将一条腿的接触力转换为关节力矩
   * @param leg_id: 要转换的腿的ID
   * @param contact_force: 腿的接触力
   * @return 关节力矩，共3个关节
   */
  std::map<int, float> MapContactForceToJointTorques(int leg_id, Vec3<float> contact_force);

  /**
   * @brief 将向量转换为不同腿的有符号向量
   * @param v: 腿上的向量
   * @param leg_id: 要转换的腿的ID
   * @return 有符号向量
   */
  Vec3<float> WithLegSigns(const Vec3<float> &v, int leg_id);

  /**
   * @brief 重置计时器
   */
  void ResetTimer() { timer.ResetStartTime(); }

  /**
   * @brief 构建机器人的动力学模型
   * @return 如果工作已完成，则返回true
   */
  virtual bool BuildDynamicModel() { return false; }

  /**
   * @brief 获取足端在机器人基坐标系下的位置
   * @return 足端在基坐标系下的位置
   */
  Mat34<float> GetFootPositionsInBaseFrame() { return stateDataFlow.footPositionsInBaseFrame; }

  /**
   * @brief 获取机器人的控制模式
   */
  std::string GetControlMode() { return modeMap[controlParams["mode"]]; }

  /**
   * @brief 获取成员baseVelocityInBaseFrame的取值
   */
  inline Vec3<float> GetBaseVelocityInBaseFrame() { return baseVelocityInBaseFrame; };

  /**
   * @brief 获取成员tick的取值
   */
  inline uint32_t GetTick() { return tick; }

  /**
   * @brief 获取成员motorAngles的取值
   */
  inline Eigen::Matrix<float, 12, 1> GetMotorAngles() { return motorAngles; };

  /**
   * @brief 获取成员motorVelocities的取值
   */
  inline Eigen::Matrix<float, 12, 1> GetMotorVelocities() const { return motorVelocities; }

  /**
   * @brief 获取成员basePosition的取值
   */
  inline Vec3<float> GetBasePosition() const { return basePosition; }

  /**
   * @brief 获取成员baseOrientation的取值
   */
  inline Eigen::Matrix<float, 4, 1> GetBaseOrientation() const { return baseOrientation; }

  /**
   * @brief 获取成员defaultHipPosition的取值
   */
  inline Mat34<float> GetDefaultHipPosition() const { return defaultHipPosition; }

  /**
   * @brief 获取成员baseRollPitchYaw的取值
   */
  inline Vec3<float> GetBaseRollPitchYaw() const { return baseRollPitchYaw; }

  /**
   * @brief 获取成员baseRollPitchYawRate的取值
   */
  inline Vec3<float> GetBaseRollPitchYawRate() const { return baseRollPitchYawRate; }

  /**
   * @brief 获取成员footForce的取值
   */
  inline Eigen::Matrix<float, 4, 1> GetFootForce() const { return footForce; }

  /**
   * @brief 获取成员footContact的取值
   */
  inline Eigen::Matrix<bool, 4, 1> GetFootContact() const { return footContact; }

  /**
   * @brief 获取成员motorKps的取值
   */
  inline Eigen::Matrix<float, 12, 1> GetMotorKps() const { return motorKps; }

  /**
   * @brief 获取成员motorKds的取值
   */
  inline Eigen::Matrix<float, 12, 1> GetMotorKdp() const { return motorKds; }

  /**
   * @brief 获取成员timeStep的取值
   */
  inline float GetTimeStep() { return timeStep; }

  /**
   * @brief 获取成员timer的取值
   */
  inline qrTimerInterface &GetTimer() { return timer; }

  /**
   * @brief 获取机器人自上次重置以来经过的时间
   * @return 重置后的时间
   */
  float GetTimeSinceReset() { return timer.GetTimeSinceReset(); }

  /**
   * @brief 机器人配置文件的路径
   */
  std::string configFilePath;

  /**
   * @brief 配置文件的YAML节点
   */
  YAML::Node robotConfig;

  /**
   * @brief 要创建的机器人类型
   */
  std::string robotName;

  /**
   * @brief 机器人的总质量
   */
  float totalMass;

  /**
   * @brief 机器人主体的质量
   */
  float bodyMass;

  /**
   * @brief 机器人的总惯量矩阵
   */
  Mat3<float> totalInertia;

  /**
   * @brief 机器人主体的惯量矩阵
   */
  Mat3<float> bodyInertia;

  /**
   * @brief 四条腿上12个关节的惯量矩阵
   */
  std::vector<Mat3<float>> linkInertias;

  /**
   * @brief 四条腿上12个关节的质量
   */
  std::vector<float> linkMasses;

  /**
   * @brief 四条腿上12个关节的长度
   */
  std::vector<float> linkLength;

  /**
   * @brief 关节中心相对于机器人基坐标系的位置
   */
  std::vector<std::vector<float>> linksComPos;

  /**
   * @brief 机器人主体的高度
   */
  float bodyHeight;

  /**
   * @brief 臀部关节相对于机器人基坐标系的位置
   */
  Vec3<float> abadLocation;

  /**
   * @brief 腿的第一关节长度
   */
  float hipLength;

  /**
   * @brief 腿的第二关节长度
   */
  float upperLegLength;

  /**
   * @brief 腿的第三关节长度
   */
  float lowerLegLength;

  /**
   * @brief 足端相对于脚关节的偏移量
   */
  float footHoldOffset = 0.1f;

  /**
   * @brief 质心相对于几何中心的偏移量
   */
  Vec3<float> comOffset;

  /**
   * @brief 臀部相对于几何中心的偏移量
   */
  Mat34<float> hipOffset;

  /**
   * @brief 默认的臀部位置
   */
  Mat34<float> defaultHipPosition;

  /**
   * @brief 关节位置控制的比例系数(单位：N.m/rad)
   */
  Eigen::Matrix<float, 12, 1> motorKps;

  /**
   * @brief 关节速度控制的比例系数(单位：N.m/(rad/s) )
   */
  Eigen::Matrix<float, 12, 1> motorKds;

  /**
   * @brief 12个关节角的方向
   * 不同的机器人可能有不同的坐标系定义
   */
  Eigen::Matrix<float, 12, 1> jointDirection = Eigen::Matrix<float, 12, 1>::Ones();

  /**
   * @brief 关节角与电机角的偏移量
   * 不同的机器人可能有不同的关节定义
   */
  Eigen::Matrix<float, 12, 1> jointOffset = Eigen::Matrix<float, 12, 1>::Zero();

  /**
   * @brief 机器人站立时的默认关节角
   */
  Eigen::Matrix<float, 12, 1> standUpMotorAngles;

  /**
   * @brief 机器人蹲下时的关节角
   */
  Eigen::Matrix<float, 12, 1> sitDownMotorAngles;

  /**
   * @brief 机器人的控制模式，
   * 包括速度、位置、步行、高级蹄态等
   */
  std::map<std::string, int> controlParams;

  /**
   * @brief 是否使用ROS时间工具
   */
  bool useRosTime = true;

  /**
   * @brief 存储机器人启动后的时间
   */
  qrTimerInterface timer;

  /**
   * @brief 控制频率
   */
  float timeStep;

  /**
   * @brief 存储上次重置机器人的时间
   */
  float lastResetTime;

  /**
   * @brief 机器人是否完成初始化
   */
  bool initComplete = false;

  /**
   * @brief 机器人是否为模拟器
   */
  bool isSim;

  /**
   * @brief 存储加速度、陀螺仪、欧拉角、四元数等变量和结果的滑动窗口滤波器
   */
  qrMovingWindowFilter<float, 3> accFilter;
  qrMovingWindowFilter<float, 3> gyroFilter;
  qrMovingWindowFilter<float, 3> rpyFilter;
  qrMovingWindowFilter<float, 4> quatFilter;
  qrMovingWindowFilter<float, 12> motorVFilter;

  /**
   * @brief 机器人基坐标系在世界坐标系下的位置
   */
  Vec3<float> basePosition = {0.f, 0.f, A1_BODY_HIGHT};

  /**
   * @brief Height of main body in world frame.
   */
  float absoluteHight = 0;

  /**
   * @brief 机器人基坐标系在世界坐标系下的方向
   */
  Eigen::Matrix<float, 4, 1> baseOrientation;

  /**
   * @brief Yaw calibrated robot rpy in world frame.
   */
  Vec3<float> baseRollPitchYaw;

  /**
   * @brief 机器人在世界坐标系下的滚转摆动角速度
   */
  Vec3<float> baseRollPitchYawRate;

  /**
   * @brief 机器人在基坐标系下的线速度
   */
  Vec3<float> baseVelocityInBaseFrame;

  /**
   * @brief 机器人在基坐标系下的加速度
   */
  Vec3<float> baseAccInBaseFrame;

  /**
   * @brief 关节的当前角度(单位：弧度)
   */
  Eigen::Matrix<float, 12, 1> motorAngles;

  /**
   * @brief 关节的当前角速度(单位：弧度/秒)
   */
  Eigen::Matrix<float, 12, 1> motorVelocities;

  /**
   * @brief 关节的当前角加速度(单位：弧度/秒^2)
   */
  Eigen::Matrix<float, 12, 1> motorddq;

  /**
   * @brief 关节的力矩(单位：N.m)
   */
  Eigen::Matrix<float, 12, 1> motortorque;

  /**
   * @brief 四个脚上的接触力(单位：N)
   */
  Eigen::Matrix<float, 4, 1> footForce;

  /**
   * @brief 四个脚的接触状态
   */
  Eigen::Matrix<bool, 4, 1> footContact;

  /**
   * @brief 存储运动学数据的对象
   */
  qrStateDataFlow stateDataFlow;

  /**
   * @brief 用于整体控制的动力学模型
   */
  FloatingBaseModel<float> model;

  /**
   * @brief 存储IMU数据、编码器数据等的对象
   */
  LowState lowState;

  /**
   * @brief 用于计算每个循环的计算时间
   */
  uint32_t tick = 0;

  /**
   * @brief 机器人启动时的偏航角
   */
  float yawOffset = 0.f;

  /**
   * @brief 机器人是否停止
   */
  bool stop = false;

  /**
   * @brief 机器人基坐标系在世界坐标系下的位置
   */
  Vec3<float> gazeboBasePosition = {0.f, 0.f, A1_BODY_HIGHT};

  /**
   * @brief 机器人基坐标系在世界坐标系下的方向
   */
  Eigen::Matrix<float, 4, 1> gazeboBaseOrientation = {1.f, 0.f, 0.f, 0.f};

  /**
   * @brief 机器人在基坐标系下的线速度
   */
  Vec3<float> gazeboBaseVInBaseFrame;

  /**
   * @brief 四个脚在世界坐标系下的位置
   */
  Mat34<float> gazeboFootPositionInWorldFrame;

  /**
   * @brief 存储控制模式与其对应的整数值的映射关系
   */
  std::unordered_map<int, std::string> modeMap = {{0, "velocity"}, {1, "position"}, {2, "walk"}, {3, "advanced_trot"}};

  /**
   * @brief 当前有限状态机的模式
   */
  int fsmMode = 4;

  /**
   * @brief 上一次电机控制模式
   */
  int lastMotorControlMode = MotorMode::POSITION_MODE;

  /**
   * @brief 上一次发送给电机的命令
   */
  Eigen::Matrix<float, 12, 5> lastMotorCommands = Eigen::Matrix<float, 12, 5>::Zero();
};

}  // Namespace Quadruped

#endif  // QR_ROBOT_H
