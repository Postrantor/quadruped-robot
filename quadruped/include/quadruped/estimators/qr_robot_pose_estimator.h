/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_POSE_ESTIMATOR_H
#define QR_ROBOT_POSE_ESTIMATOR_H

#include "quadruped/estimators/qr_ground_surface_estimator.h"
#include "quadruped/estimators/qr_robot_velocity_estimator.h"
#include "quadruped/gait/qr_openloop_gait_generator.h"
#include "quadruped/robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief 估算机器人姿态，当前代码实现了x轴和y轴上的位置估算。
 */
class qrRobotPoseEstimator {
public:
  /**
   * @brief 机器人姿态估算器的构造函数。
   * @param robotIn：机器人类用于姿态估算。
   * @param gaitGeneratorIn：生成所需步态计划用于行走。
   * @param groundEstimatorIn：估算脚部接触的3D平面。
   * @param velocityEstimator：速度估算类用于速度估算。
   */
  qrRobotPoseEstimator(
      qrRobot* robotIn,
      qrGaitGenerator* gaitGeneratorIn,
      qrGroundSurfaceEstimator* groundEstimatorIn,
      qrRobotVelocityEstimator* velocityEstimator);

  /**
   * @brief 重置估算器
   * @param currentTime：计时器启动以来的时间。
   */
  void Reset(float currentTime);

  /**
   * @brief 计算每个循环的时间间隔。
   * @param tick：真实机器人状态用于计算每个循环的时间。
   * @return 每个循环的时间间隔。
   */
  float ComputeDeltaTime(uint32_t tick);

  /**
   * @brief 更新机器人的姿态。
   * @param currentTime：计时器启动以来的时间。
   */
  void Update(float currentTime);

  /**
   * @brief 计算机器人的高度。
   * @return 估算的机器人高度。
   */
  float EstimateRobotHeight();

  /**
   * @brief 简单地计算里程表并更新机器人姿态(x,y,yaw)。
   * @param deltaTime：每个循环的时间间隔。
   */
  void ComputePose(float deltaTime);

  /**
   * @brief 获取估算姿态的Getter方法。
   */
  const Vec6<float>& GetEstimatedPose() const { return estimatedPose; }

private:
  /**
   * @brief 机器人类用于姿态估算。
   */
  qrRobot* robot;

  /**
   * @brief 上一个循环完成的时间戳。
   */
  uint32_t lastTimestamp;

  /**
   * @brief 估算的机器人姿态。
   */
  Vec6<float> estimatedPose;

  /**
   * @brief 生成所需步态计划用于行走。
   */
  qrGaitGenerator* gaitGenerator;

  /**
   * @brief 估算脚部接触的3D平面。
   */
  qrGroundSurfaceEstimator* groundEstimator;

  /**
   * @brief 速度估算类用于姿态估算。
   */
  qrRobotVelocityEstimator* velocityEstimator;
};

}  // Namespace Quadruped

#endif  // QR_ROBOT_POSE_ESTIMATOR_H
