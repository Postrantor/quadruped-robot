/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_ESTIMATOR_H
#define QR_ROBOT_ESTIMATOR_H

#include "estimators/qr_ground_surface_estimator.h"
#include "estimators/qr_robot_pose_estimator.h"
#include "estimators/qr_robot_velocity_estimator.h"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"

/* CMU paramaters */
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001
#define STATE_SIZE 18
#define MEAS_SIZE 28

namespace Quadruped {

/**
 * @brief 估算机器人状态，包括姿态和速度。
 */
class qrRobotEstimator : public qrBaseStateEstimator {
public:
  /**
   * @brief 估算机器人状态，包括姿态和速度。
   * @param robotIn: 机器人类用于估算。
   * @param gaitGeneratorIn: 由 gait 调度器生成期望步态。
   * @param groundEstimatorIn: 估算脚部接触的 3D 平面。
   * @param userParametersIn: 存储估算器配置参数。
   */
  qrRobotEstimator(
      qrRobot *robotIn,
      qrGaitGenerator *gaitGeneratorIn,
      qrGroundSurfaceEstimator *groundEstimatorIn,
      qrUserParameters *userParametersIn);

  /**
   * @brief 重置估算器。
   * @param currentTime: 计时器启动以来当前时间。
   */
  void Reset(float currentTime);

  /**
   * @brief 计算每个循环的时间间隔。
   * @param tick: 实际机器人状态，用于计算每个循环的时间。
   */
  float ComputeDeltaTime(uint32_t tick);

  /**
   * @brief 计算和更新零矩点。
   * @return 零矩点的向量。
   */
  Vec3<float> ComputeZMP();

  /**
   * @brief 更新机器人运动学状态。
   * @param currentTime: 计时器启动以来当前时间。
   */
  void Update(float currentTime);

  /**
   * @brief 初始化 Filter 状态。
   */
  void CMUInitState();

  /**
   * @brief 使用 Filter 更新脚部接触状态和接触力。
   * @param currentTime: 计时器启动以来当前时间。
   */
  void CMUUpdate(double currentTime);

  /**
   * @brief estimatedVelocity 的 getter 方法。
   */
  inline const Vec3<float> &GetEstimatedVelocity() const { return estimatedVelocity; };

  /**
   * @brief estimatedAngularVelocity 的 getter 方法。
   */
  inline const Vec3<float> &GetEstimatedAngularVelocity() const { return estimatedAngularVelocity; };

  /**
   * @brief estimatedPosition 的 getter 方法。
   */
  inline const Vec3<float> &GetEstimatedPosition() const { return estimatedPosition; };

  /**
   * @brief estimatedRPY 的 getter 方法。
   */
  inline const Vec3<float> &GetEstimatedRPY() { return estimatedRPY; };

private:
  /**
   * @brief 估算机器人的机器人类。
   */
  qrRobot *robot;

  /**
   * @brief 机器人的速度估算器。
   */
  qrRobotVelocityEstimator velocityEstimator;

  /**
   * @brief 机器人的姿态估算器。
   */
  qrRobotPoseEstimator poseEstimator;

  /**
   * @brief 计时器重置以来的时间。
   */
  float timeSinceReset;

  /**
   * @brief 世界坐标系中的估算位置。
   */
  Vec3<float> estimatedPosition;

  /**
   * @brief 世界坐标系中的估算 roll-pitch-yaw。
   */
  Vec3<float> estimatedRPY;

  /**
   * @brief 底座坐标系中的估算速度。
   */
  Vec3<float> estimatedVelocity;

  /**
   * @brief 底座坐标系中的上一次估算速度。
   */
  Vec3<float> lastEstimatedVelocity;

  /**
   * @brief 世界坐标系中的估算角速度。
   */
  Vec3<float> estimatedAngularVelocity;

  /**
   * @brief 上一个真实机器人状态用于计算每个循环的时间。
   */
  uint32_t lastTimestamp;

  /**
   * @brief 估算状态。
   * 0 1 2 pos 3 4 5 vel 6 7 8 足部 pos FL 9 10 11 足部 pos FR 12 13 14 足部 pos RL 15 16 17 足部 pos RR
   */
  Eigen::Matrix<double, STATE_SIZE, 1> x;  // 估算状态

  /**
   * @brief 过程更新后的估算状态。
   */
  Eigen::Matrix<double, STATE_SIZE, 1> xbar;  // 过程更新后的估算状态

  /**
   * @brief 估算状态协方差。
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;  // 估算状态协方差

  /**
   * @brief 过程更新后的估算状态协方差。
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar;  // 过程更新后的估算状态协方差

  /**
   * @brief 估算状态转换。
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A;  // 估算状态转换

  /**
   * @brief 估算状态转换。
   */
  Eigen::Matrix<double, STATE_SIZE, 3> B;  // 估算状态转换

  /**
   * @brief 估算状态转换噪声。
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;  // 估算状态转换噪声

  /**
   * @brief 观测。
   * 观测
   * 0 1 2   FL 位置残差
   * 3 4 5   FR 位置残差
   * 6 7 8   RL 位置残差
   * 9 10 11 RR 位置残差
   * 12 13 14 速度残差来自 FL
   * 15 16 17 速度残差来自 FR
   * 18 19 20 速度残差来自 RL
   * 21 22 23 速度残差来自 RR
   * 24 25 26 27 脚部高度
   */
  Eigen::Matrix<double, MEAS_SIZE, 1> y;

  /**
   * @brief 估算观测。
   */
  Eigen::Matrix<double, MEAS_SIZE, 1> yhat;

  /**
   * @brief 估算观测。
   */
  Eigen::Matrix<double, MEAS_SIZE, 1> error_y;

  /**
   * @brief S^-1*error_y。
   */
  Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;

  /**
   * @brief 估算状态观测。
   */
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;

  /**
   * @brief S^-1*C。
   */
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC;

  /**
   * @brief 估算状态观测噪声
   */
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;

  /**
   * @brief 3x3 单位矩阵。
   */
  Eigen::Matrix<double, 3, 3> eye3;

  /**
   * @brief 创新（或预适残差）协方差。
   */
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;

  /**
   * @brief 卡尔曼增益。
   */
  Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K;

  /**
   * @brief 是否假设地面是平的。
   */
  bool assume_flat_ground = true;

  /**
   * @brief 最大滑动摩擦。
   */
  double smooth_foot_force[4];

  /**
   * @brief  estimated 接触状态。
   */
  double estimated_contacts[4];
};

}  // Namespace Quadruped

#endif  // QR_ROBOT_ESTIMATOR_H
