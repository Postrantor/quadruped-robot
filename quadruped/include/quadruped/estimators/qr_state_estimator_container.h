/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STATE_ESTIMATOR_CONTAINER_H
#define QR_STATE_ESTIMATOR_CONTAINER_H

#include "quadruped/estimators/qr_anomaly_detection.h"
#include "quadruped/estimators/qr_base_state_estimator.h"
#include "quadruped/estimators/qr_ground_surface_estimator.h"
#include "quadruped/estimators/qr_robot_estimator.h"

namespace Quadruped {

/**
 * @brief 主状态估算器类
 *   包含所有泛型估算器，并可以运行它们
 *   同时更新可视化效果。
 */
class qrStateEstimatorContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 状态估算器容器的构造函数
   * @param quadrupedIn：机器人用于状态估算
   * @param gaitGeneratorIn：生成所需步态计划用于行走
   * @param userParametersIn：卡尔曼滤波器和移动窗口算法的参数
   * @param terrainConfigPath：地形配置文件路径
   */
  qrStateEstimatorContainer(
      qrRobot* quadrupedIn,
      qrGaitGenerator* gaitGeneratorIn,
      qrUserParameters* userParametersIn,
      std::string terrainConfigPath);

  /**
   * @brief 重置所有包含的泛型估算器
   */
  void Reset() {
    resetTime = 0;
    timeSinceReset = 0.;
    groundEstimator->Reset(timeSinceReset);
    contactDetection->Reset(timeSinceReset);
    robotEstimator->Reset(timeSinceReset);
    std::cout << "StateEstimatorContainer Reset" << std::endl;
  }

  /**
   * @brief 更新所有包含的泛型估算器
   */
  void Update() {
    timeSinceReset = quadruped->GetTimeSinceReset() - resetTime;
    // contactDetection->Update(timeSinceReset);
    groundEstimator->Update(timeSinceReset);
    robotEstimator->Update(timeSinceReset);
  }

  /**
   * @brief 删除所有包含的泛型估算器
   */
  void RemoveAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  /**
   * @brief 状态估算器容器的析构函数
   */
  ~qrStateEstimatorContainer() = default;

  /**
   * @brief 获取contactDetection类用于状态估算
   */
  inline qrContactDetection* GetContactDetection() { return contactDetection; }

  /**
   * @brief 获取robotEstimator类用于状态估算
   */
  inline qrRobotEstimator* GetRobotEstimator() { return robotEstimator; }

  /**
   * @brief 获取GroundSurfaceEstimator类用于状态估算
   */
  inline qrGroundSurfaceEstimator* GetGroundEstimator() { return groundEstimator; }

private:
  /**
   * @brief 机器人用于状态估算
   */
  qrRobot* quadruped;

  /**
   * @brief 步态计划生成器用于状态估算
   */
  qrGaitGenerator* gaitGenerator;

  /**
   * @brief 卡尔曼滤波器和移动窗口算法的参数
   */
  qrUserParameters* userParameters;

  /**
   * @brief 存储所有包含的泛型估算器
   */
  std::vector<qrBaseStateEstimator*> _estimators;

  /**
   * @brief 估算接触平面
   */
  qrGroundSurfaceEstimator* groundEstimator;

  /**
   * @brief 接触检测类用于状态估算
   */
  qrContactDetection* contactDetection;

  /**
   * @brief 机器人估算类用于状态估算
   */
  qrRobotEstimator* robotEstimator;

  /**
   * @brief 计时器重启的时间
   */
  float resetTime;

  /**
   * @brief 计时器重启以来的时间
   */
  float timeSinceReset;
};

}  // namespace Quadruped

#endif  // QR_STATE_ESTIMATOR_CONTAINER_H
