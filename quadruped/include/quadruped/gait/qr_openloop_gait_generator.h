/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_OPENLOOP_GAIT_GENERATOR_H
#define QR_OPENLOOP_GAIT_GENERATOR_H

#include <math.h>

#include <vector>

#include "quadruped/gait/qr_gait.h"
#include "quadruped/robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief 循环策略的步态生成器。
 */
class qrOpenLoopGaitGenerator : public qrGaitGenerator {
public:
  /**
   * @brief OpenloopGaitGenerator 类的构造函数。
   */
  qrOpenLoopGaitGenerator();

  /**
   * @brief OpenloopGaitGenerator 类的构造函数。
   * @param robot: 生成所需步态的机器人。
   * @param configFilePath: 给定的配置文件路径。
   */
  qrOpenLoopGaitGenerator(qrRobot* robot, std::string configFilePath);

  /**
   * @brief 通过直接参数构造 OpenloopGaitGenerator 类。
   * @param robot: 生成所需步态的机器人。
   * @param stanceDuration: stanceDuration 指定每个腿在步态周期中的支撑时间。
   * @param dutyFactor: dutyFactor 指定每个腿的占空比。dutyFactor 代表步态周期中的支撑相位部分。
   * @param initialLegState: initialLegState 指定每个腿在生成步态时的初始状态（SWING 或 STANCE）。
   * @param initialLegPhase: initialLegPhase 指定每个腿在生成步态时的相位。
   * @param contactDetectionPhaseThreshold: contactDetectionPhaseThreshold 指定腿状态从 SWING 到 STANCE 的检测阈值。
   */
  qrOpenLoopGaitGenerator(
      qrRobot* robot,
      Eigen::Matrix<float, 4, 1> stanceDuration,
      Eigen::Matrix<float, 4, 1> dutyFactor,
      Eigen::Matrix<int, 4, 1> initialLegState,
      Eigen::Matrix<float, 4, 1> initialLegPhase,
      float contactDetectionPhaseThreshold = 0.1f);

  /**
   * @brief OpenloopGaitGenerator 类的析构函数。
   */
  virtual ~qrOpenLoopGaitGenerator() = default;

  /**
   * @brief 使用给定的时间重置步态参数。
   * @param currentTime: 给定的时间。
   */
  virtual void Reset(float currentTime);

  /**
   * @brief 使用给定的时间更新步态参数。
   * @param currentTime: 给定的时间。
   */
  virtual void Update(float currentTime);

  /**
   * @brief 如果脚部失去接触，等待一秒或在接触时允许切换腿状态。
   * @param currentTime: 给定的时间。
   */
  virtual void Schedule(float currentTime);

private:
  /**
   * @brief 累积的 delta 时间，用于调度。
   */
  float cumDt = 0;

  /**
   * @brief 等待步态切换的时间。
   */
  float waitTime = 1.0;
};
}  // Namespace Quadruped

#endif  // QR_OPENLOOP_GAIT_GENERATOR_H
