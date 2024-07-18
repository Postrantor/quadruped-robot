/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_GAIT_GENERATOR_H
#define QR_GAIT_GENERATOR_H

#include "quadruped/robots/qr_robot.h"

/**
 * @brief qrGaitGenerator 类生成相应的步态，使用定义的步态参数。该类还定期重置和更新这些参数。
 */
namespace Quadruped {

class qrGaitGenerator {
public:
  /**
   * @brief 默认构造函数，构造 qrGaitGenerator 对象。
   */
  qrGaitGenerator() {}

  /**
   * @brief 使用给定的配置文件构造 qrGaitGenerator 对象。
   * @param robot: 机器人状态类。
   * @param configFilePath: 给定的配置文件。
   */
  qrGaitGenerator(qrRobot* robot, std::string configFilePath);

  /**
   * @brief 使用给定的参数构造 qrGaitGenerator 对象。
   * @param robot: 机器人状态类。
   * @param stanceDuration: 指定每条腿在步态周期中的stance时间。
   * @param dutyFactor: 指定每条腿的-dutyFactor，表示步态周期中的stance相对时间。
   * @param initialLegState: 指定每条腿在生成步态时的初始状态（SWING 或 STANCE）。
   * @param initialLegPhase: 指定每条腿在生成步态时的相对相位。
   * @param contactDetectionPhaseThreshold: 指定腿状态从 SWING 到 STANCE 的触摸检测阈值。
   */
  qrGaitGenerator(
      qrRobot* robot,
      Eigen::Matrix<float, 4, 1> stanceDuration,
      Eigen::Matrix<float, 4, 1> dutyFactor,
      Eigen::Matrix<int, 4, 1> initialLegState,
      Eigen::Matrix<float, 4, 1> initialLegPhase,
      float contactDetectionPhaseThreshold = 0.1f);

  /**
   * @brief 析构 qrGaitGenerator 对象。
   */
  virtual ~qrGaitGenerator() = default;

  /**
   * @brief 使用给定的时间重置步态参数。
   * @param currentTime 给定的时间。
   */
  virtual void Reset(float currentTime) {
    resetTime = currentTime;
    lastTime = currentTime;
    normalizedPhase.setZero();
    curLegState = initialLegState;
    lastLegState = curLegState;
    legState = curLegState;
    desiredLegState = curLegState;
    firstStanceAngles = robot->standUpMotorAngles;
    contactStartPhase.setZero();
  }

  /**
   * @brief 使用给定的时间更新步态参数。
   * @param currentTime 给定的时间。
   */
  virtual void Update(float currentTime) = 0;

  /**
   * @brief 如果脚部不在期望状态中，该如何计划下一个期望状态。
   * @param currentTime 给定的时间。
   */
  virtual void Schedule(float currentTime) {}

  /**
   * @brief yaml 对象，用于加载 yaml 配置文件。
   */
  YAML::Node config;

  /**
   * @brief 机器人对象。
   */
  qrRobot* robot;

  /**
   * @brief 步态名称。
   */
  std::string gait;

  /**
   * @brief 执行 reset 的时间。
   */
  float resetTime;

  /**
   * @brief 自 exec RESET 以来的时间。
   */
  float timeSinceReset;

  /**
   * @brief 执行 RESET 的时间。
   */
  float lastTime;

  /**
   * @brief 每条腿在步态周期中的stance时间。
   */
  Eigen::Matrix<float, 4, 1> stanceDuration;

  /**
   * @brief 每条腿在步态周期中的swing时间。
   */
  Eigen::Matrix<float, 4, 1> swingDuration;

  /**
   * @brief 步态周期中的stance相对时间。
   * dutyFactor = stanceDuration / (stanceDuration + swingDuration)。
   * @note 在周期性步态中，dutyFactor 对于所有腿都是相同的。
   */
  Eigen::Matrix<float, 4, 1> dutyFactor;

  /**
   * @brief 每条腿在完整步态周期中的当前相位。
   */
  Eigen::Matrix<float, 4, 1> phaseInFullCycle;

  /**
   * @brief 生成步态时每条腿的相对相位。
   * @note 一条腿被分配了相对相位 0，其他腿的相对相位在 [0,1) 范围内。
   */
  Eigen::Matrix<float, 4, 1> initialLegPhase;

  /**
   * @brief 全局初始腿相位，[0, stance duty] 为 stance，[stance duty, 1.0] 为 swing。
   */
  Eigen::Matrix<float, 4, 1> offset;

  /**
   * @brief 当前腿状态切换时每条腿的新状态，或者从 STAND 切换到 SWING，或者从 SWING 切换到 STAND。
   * @note 如果当前状态是 SWING，则下一个状态将是 STAND。如果当前状态是 STANCE，则下一个状态将是 SWING。
   */
  Eigen::Matrix<int, 4, 1> initialLegState;

  /**
   * @brief 当前腿状态切换时每条腿的新状态，或者从 STAND 切换到 SWING，或者从 SWING 切换到 STAND。
   * @note 如果当前状态是 SWING，则下一个状态将是 STAND。如果当前状态是 STANCE，则下一个状态将是 SWING。
   */
  Eigen::Matrix<int, 4, 1> nextLegState;

  /**
   * @brief 每条腿的当前状态（either STANCE or SWING）。
   */
  Eigen::Matrix<int, 4, 1> legState;

  /**
   * @brief 期望状态的相对相位。
   * @note 如果当前状态是 SWING，则归一化腿相位表示（time in swing）/（总 swing 时间），类似于 STANCE。
   */
  Eigen::Matrix<float, 4, 1> normalizedPhase;

  /**
   * @brief 在给定时刻每条腿的期望状态（either STANCE or SWING）。
   * 例如，如果 legState = STANCE 且 normalizedLegPhase < 1.0，则 desiredLegState = legState。
   * 例如，如果 legState = STANCE 且 normalizedLegPhase >= 1.0，则 desiredLegState = nextLegState。
   */
  Eigen::Matrix<int, 4, 1> desiredLegState;

  /**
   * @brief 每条腿的前一个状态，
   * @note 这用于确定 Velocity Mode 中的状态是否改变。
   */
  Eigen::Matrix<int, 4, 1> lastLegState;

  /**
   * @brief 每条腿的前一个状态，
   * @note 这用于确定在 Position Mode 或 Walk Mode 中状态是否改变。
   */
  Eigen::Matrix<int, 4, 1> curLegState;  // curLegState 是计划的当前状态，而 legState 是通过传感器检测到的状态。

  /**
   * @brief 步态周期中 stance 相位或 swing 相位的分数。
   * @note 如果 legState = STANCE，则值为 dutyfactor；如果 legState = SWING，则值为 (1 - dutyfactor)。
   */
  Eigen::Matrix<float, 4, 1> initStateRadioInCycle;

  /**
   * @brief 步态周期的持续时间。fullCyclePeriod = stanceDuration + swingDuration。
   */
  Vec4<float> fullCyclePeriod;

  /**
   * @brief 是否允许切换腿状态。
   */
  Vec4<bool> allowSwitchLegState;

  /**
   * @brief_legState 从 SWING 切换到 STANCE 时的接触阈值。
   */
  float contactDetectionPhaseThreshold;

  /**
   * @brief 百分之几的周期内移动基础。
   */
  std::vector<float> moveBaseRatioPoint;

  /**
   * @brief 在 stance 期间移动躯干（基础）的时间。
   */
  float moveBaseTime;

  /**
   * @brief 移动基础的相对时间。
   */
  float moveBasePhase;

  /**
   * @brief 在 swing 周期中 'trueSwing' 步态开始的初始相位。
   * @note 一个完整的周期包括 swing 和 stance，另外，stance 期间包括 'load_force' 'full_stance' 'unload_stance'。
   */
  float trueSwingStartPhaseInSwingCycle;

  /**
   * @brief 在完整周期中 'trueSwing' 步态开始的初始时间。
   */
  float trueSwingStartPhaseInFullCycle;

  /**
   * @brief 在完整周期中 'trueSwing' 步态结束的时间。
   */
  float trueSwingEndPhaseInFullCycle;

  /**
   * @brief 生成了多少个步态周期。
   */
  unsigned long gaitCycle = 0;

  /**
   * @brief 机器人检测到的腿状态，例如，如果腿在 swing 步态时与地面接触，则腿可能处于 '早期接触' 状态。
   */
  Eigen::Matrix<int, 4, 1> detectedLegState;

  /**
   * @brief 在完整周期中事件（早期接触或失去接触）发生的时间。
   */
  Vec4<float> detectedEventTickPhase;

  /**
   * @brief 是否应该将腿状态更改为 swing。
   */
  Vec4<bool> firstSwing = {false, false, false, false};

  /**
   * @brief 腿在完整周期中的接触时间。
   */
  Vec4<float> contactStartPhase;

  /**
   * @brief 当真实 swing 结束时剩余的时间。
   */
  Vec4<float> swingTimeRemaining = {0.f, 0.f, 0.f, 0.f};

  /**
   * @brief 腿是否完成了第一个 stance 步态。
   */
  Vec4<bool> firstStance = {false, false, false, false};

  /**
   * @brief 机器人 stance 步态的目标关节角度。
   */
  Eigen::Matrix<float, 12, 1> firstStanceAngles;
};

}  // Namespace Quadruped

#endif  // QR_GAIT_GENERATOR_H
