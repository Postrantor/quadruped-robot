/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_VELOCITY_ESTIMATOR_H
#define QR_ROBOT_VELOCITY_ESTIMATOR_H

#include <deque>
#include <numeric>

#include "quadruped/estimators/qr_moving_window_filter.hpp"
#include "quadruped/gait/qr_openloop_gait_generator.h"
#include "quadruped/robots/qr_robot.h"
#include "quadruped/utils/qr_se3.h"

/* 这是一个外部库，但使用了qr_filter.hpp中定义的变量*/
#include "TinyEKF.h"

namespace Quadruped {

/** @brief 初始化速度估算器。
 *  查看filterpy文档了解更多详情。
 *  https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html。
 * @param robot：机器人类用于速度估算。
 * @param accelerometer_variance：加速度计读取的噪声估算。
 * @param sensor_variance：马达速度读取的噪声估算。
 * @param initial_covariance：初始状态的协方差估算。
 */
class qrRobotVelocityEstimator {
public:
  /**
   * @brief 估算A1机器人的基座速度。
   * 速度估算器由两部分组成：
   * 1)中心质量速度状态估算器。
   *
   * 两个信息源被用于估算：
   * 加速度计读取和接触腿部的速度估算。
   * 读取被使用卡尔曼滤波器融合在一起。
   *
   * 2)一个移动平均滤波器来平滑速度读取
   * @param robot：机器人类用于速度估算。
   * @param gaitGeneratorIn：生成所需步态计划用于行走。
   * @param userParametersIn：卡尔曼滤波器和移动窗口算法的参数。
   */
  qrRobotVelocityEstimator(qrRobot *robot, qrGaitGenerator *gaitGeneratorIn, qrUserParameters *userParametersIn);

  /**
   * @brief 重置机器人速度估算器。
   * @param currentTime：计时器启动以来的时间。
   */
  void Reset(float currentTime);

  /**
   * @brief 计算两个相邻imu消息之间的时间间隔。
   */
  float ComputeDeltaTime(uint32_t tick);

  /**
   * @brief 估算基座坐标系下的速度。
   */
  void Update(float currentTime);

  /**
   * @brief 获取成员estimatedVelocity的Getter方法。
   */
  const Vec3<float> &GetEstimatedVelocity() const { return estimatedVelocity; }

  /**
   * @brief 获取成员estimatedAngularVelocity的Getter方法。
   */
  const Vec3<float> &GetEstimatedAngularVelocity() const { return estimatedAngularVelocity; }

private:
  /**
   * @brief 机器人类用于速度估算。
   */
  qrRobot *robot;

  /**
   * @brief 生成所需步态计划用于行走。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 移动窗口算法的窗口大小。
   */
  int windowSize;

  /**
   * @brief 卡尔曼滤波器的初始方差。
   */
  float initialVariance;

  /**
   * @brief 上一个循环结束的时间戳。
   */
  uint32_t lastTimestamp;

  /**
   * @brief 基座坐标系下的估算速度。
   */
  Vec3<float> estimatedVelocity;

  /**
   * @brief 基座坐标系下的估算角速度。
   */
  Vec3<float> estimatedAngularVelocity;

  /**
   * @brief 卡尔曼滤波器的移动窗口滤波器（x轴）。
   */
  qrMovingWindowFilter<double, 1> velocityFilterX;

  /**
   * @brief 卡尔曼滤波器的移动窗口滤波器（y轴）。
   */
  qrMovingWindowFilter<double, 1> velocityFilterY;

  /**
   * @brief 卡尔曼滤波器的移动窗口滤波器（z轴）。
   */
  qrMovingWindowFilter<double, 1> velocityFilterZ;

  /**
   * @brief 卡尔曼滤波器的移动窗口滤波器（线加速度）。
   */
  qrMovingWindowFilter<float, 3> AccFilter;

  /**
   * @brief 卡尔曼滤波器用于速度估算。
   */
  TinyEKF<3, 3> *filter;
};

}  // namespace Quadruped

#endif  // QR_ROBOT_VELOCITY_ESTIMATOR_H
