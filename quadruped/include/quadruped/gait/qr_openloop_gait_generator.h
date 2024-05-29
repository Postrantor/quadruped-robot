/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_OPENLOOP_GAIT_GENERATOR_H
#define QR_OPENLOOP_GAIT_GENERATOR_H

#include <math.h>

#include <vector>

#include "gait/qr_gait.h"
#include "robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief A cyclic stratage for gait locomotion.
 */
class qrOpenLoopGaitGenerator : public qrGaitGenerator {
public:
  /**
   * @brief Constructor of OpenloopGaitGenerator class.
   */
  qrOpenLoopGaitGenerator();

  /**
   * @brief Constructor of OpenloopGaitGenerator class.
   * @param robot: robot for generate desired gait.
   * @param configFilePath: configFilePath the given config file.
   */
  qrOpenLoopGaitGenerator(qrRobot *robot, std::string configFilePath);

  /**
   * @brief Constructor of OpenloopGaitGenerator class by directly parameters.
   * @param robot: robot for generate desired gait.
   * @param stanceDuration: stanceDuration specifies the amount of stance time for each leg in a gait cycle.
   * @param dutyFactor: dutyFactor specifies the duty factor for each leg. dutyFactor represents the fraction of stance
   * phase in the gait cycle.
   * @param initialLegState: initialLegState specifies the state (SWING or STANCE) of each leg at the initialization of
   * generating a gait.
   * @param initialLegPhase: initialLegPhase specifies the relative phase for each leg at the initialization of
   * generating a gait.
   * @param contactDetectionPhaseThreshold: contactDetectionPhaseThreshold specifies the contact threshold when the leg
   * state switches from SWING to STANCE.
   */
  qrOpenLoopGaitGenerator(
      qrRobot *robot,
      Eigen::Matrix<float, 4, 1> stanceDuration,
      Eigen::Matrix<float, 4, 1> dutyFactor,
      Eigen::Matrix<int, 4, 1> initialLegState,
      Eigen::Matrix<float, 4, 1> initialLegPhase,
      float contactDetectionPhaseThreshold = 0.1f);

  /**
   * @brief Destructor of the OpenloopGaitGenerator class.
   */
  virtual ~qrOpenLoopGaitGenerator() = default;

  /**
   * @brief Reset the gait parameters using the given time.
   * @param currentTime the given time.
   */
  virtual void Reset(float currentTime);

  /**
   * @brief Update of the gait parameters using the given time.
   * @param currentTime: the given time.
   */
  virtual void Update(float currentTime);

  /**
   * @brief If feet lost contact, wait a second or it is in contact to allow switch leg state.
   * @param currentTime: the given time.
   */
  virtual void Schedule(float currentTime);

private:
  /**
   * @brief Sum of delta time,used for schedule.
   */
  float cumDt = 0;

  /**
   * @brief How long waitting for gait switch.
   */
  float waitTime = 1.0;
};
}  // Namespace Quadruped

#endif  // QR_OPENLOOP_GAIT_GENERATOR_H
