/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STATE_ESTIMATOR_CONTAINER_H
#define QR_STATE_ESTIMATOR_CONTAINER_H

#include "estimators/qr_anomaly_detection.h"
#include "estimators/qr_base_state_estimator.h"
#include "estimators/qr_ground_surface_estimator.h"
#include "estimators/qr_robot_estimator.h"

namespace Quadruped {

/**
 * @brief Main State Estimator Class
 *   Contains all GenericEstimators, and can run them
 *   Also updates visualizations.
 */
class qrStateEstimatorContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor of state Estimator container
   * @param quadrupedIn: the robot for state estimation
   * @param gaitGeneratorIn: generate desired gait schedule for locomotion
   * @param userParametersIn: parameters for kalman filter and moving window algorithm
   * @param terrainConfigPath: terrian config file path
   //!warning  homeDir: this is not used
      */
  qrStateEstimatorContainer(
      qrRobot *quadrupedIn,
      qrGaitGenerator *gaitGeneratorIn,
      qrUserParameters *userParametersIn,
      std::string terrainConfigPath,
      std::string homeDir);

  /**
   * @brief reset all contained GenericEstimators
   */
  void Reset() {
    resetTime = 0;
    timeSinceReset = 0.;
    groundEstimator->Reset(timeSinceReset);
    contactDetection->Reset(timeSinceReset);
    robotEstimator->Reset(timeSinceReset);
    std::cout << "StateEstimatorContainer Reset" << std::endl;
  };

  /**
   * @brief update all contained GenericEstimators
   */
  void Update() {
    timeSinceReset = quadruped->GetTimeSinceReset() - resetTime;
    // contactDetection->Update(timeSinceReset);
    groundEstimator->Update(timeSinceReset);
    robotEstimator->Update(timeSinceReset);
  };

  /**
   * @brief remove all contained GenericEstimators
   */
  void RemoveAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  };

  /**
   * @brief destructor of the state estimator container
   */
  ~qrStateEstimatorContainer() = default;

  /**
   * @brief get the contactDection class for the state estimation
   */
  inline qrContactDetection *GetContactDetection() { return contactDetection; };

  /**
   * @brief get the robotEstimator class for the state estimation
   */
  inline qrRobotEstimator *GetRobotEstimator() { return robotEstimator; };

  /**
   * @brief get the GroundSurfaceEstimator class for the state estimation
   */
  inline qrGroundSurfaceEstimator *GetGroundEstimator() { return groundEstimator; };

private:
  /**
   * @brief robot for state estimation
   */
  qrRobot *quadruped;

  /**
   * @brief the gait schedular for state estimation
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief parameters for kalman filter and moving window algorithm
   */
  qrUserParameters *userParameters;

  /**
   * @brief stores all contained GenericEstimators
   */
  std::vector<qrBaseStateEstimator *> _estimators;

  /**
   * @brief estimate the 3D plane where the feet contact
   */
  qrGroundSurfaceEstimator *groundEstimator;

  /**
   * @brief Contact detection class for state estimation
   */
  qrContactDetection *contactDetection;

  /**
   * @brief robot estimation class for state estimation
   */
  qrRobotEstimator *robotEstimator;

  /**
   * @brief the time when the timer restarted
   */
  float resetTime;

  /**
   * @brief the time since the timer restarted
   */
  float timeSinceReset;
};

}  // namespace Quadruped

#endif  // QR_STATE_ESTIMATOR_CONTAINER_H
