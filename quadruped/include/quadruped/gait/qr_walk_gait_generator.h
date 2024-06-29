/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_WALK_GAIT_GENERATOR_H
#define QR_WALK_GAIT_GENERATOR_H

#include "gait/qr_gait.h"
#include "robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief 步行模式下的步态生成器。
 */
class qrWalkGaitGenerator : public qrGaitGenerator {
public:
  /**
   * @brief WalkGaitGenerator 类的构造函数。
   */
  qrWalkGaitGenerator(){};

  /**
   * @brief WalkGaitGenerator 类的构造函数。
   * @param robot: 生成所需步态的机器人。
   * @param configFilePath: 给定的配置文件路径。
   */
  qrWalkGaitGenerator(qrRobot *robot, std::string configFilePath);

  /**
   * @brief WalkGaitGenerator 类的构造函数。
   * @param robot: 机器人状态的类。
   * @param stateSwitchQue: 存储腿子状态的顺序。
   * @param stateRatioQue: 存储每个腿子状态在周期中的时间比例。
   * @param stanceDuration: stanceDuration 指定每个腿在步态周期中的支撑时间。
   * @param dutyFactor: dutyFactor 指定每个腿的占空比。dutyFactor 代表步态周期中的支撑相位部分。
   * @param initialLegState: initialLegState 指定每个腿在生成步态时的初始状态（SWING 或 STANCE）。
   * @param initialLegPhase: initialLegPhase 指定每个腿在生成步态时的相位。
   * @param contactDetectionPhaseThreshold: contactDetectionPhaseThreshold 指定腿状态从 SWING 到 STANCE 的检测阈值。
   */
  qrWalkGaitGenerator(
      qrRobot *robot,
      std::vector<SubLegState> stateSwitchQue,
      std::vector<float> stateRatioQue,
      Eigen::Matrix<float, 4, 1> stanceDuration,
      Eigen::Matrix<float, 4, 1> dutyFactor,
      Eigen::Matrix<int, 4, 1> initialLegState,
      Eigen::Matrix<float, 4, 1> initialLegPhase,
      float contactDetectionPhaseThreshold = 0.1f);

  /**
   * @brief WalkGaitGenerator 类的析构函数，应该从站立状态开始。
   */
  virtual ~qrWalkGaitGenerator() = default;

  /**
   * @brief 使用给定的时间重置步态参数。
   * @param currentTime 给定的时间。
   */
  virtual void Reset(float currentTime);

  /**
   * @brief 使用给定的时间更新步态参数和腿状态。
   * @param currentTime 给定的时间。
   */
  virtual void Update(float currentTime);

  // private:

  /**
   * @brief 存储腿子状态的顺序，应该从站立状态开始。
   */
  std::vector<SubLegState> stateSwitchQue;

  /**
   * @brief 存储每个腿子状态在周期中的时间比例。
   */
  std::vector<float> stateRatioQue;

  /**
   * @brief 完整步态周期中的状态比例累积。
   */
  std::vector<float> accumStateRatioQue;

  /**
   * @brief stateSwitchQue 中的索引，用于改变子状态。
   */
  Vec4<int> stateIndexOfLegs;

  /**
   * @brief 当前周期中的子状态比例。
   */
  Vec4<float> curStateRadio;

  /**
   * @brief "trueSwing" 子状态在摆动周期中的启动相位。
   */
  float trueSwingStartPhaseInSwingCycle;
};
}  // Namespace Quadruped

#endif  // QR_WALK_GAIT_GENERATOR_H
