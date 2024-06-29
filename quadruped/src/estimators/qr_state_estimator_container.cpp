/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "estimators/qr_state_estimator_container.h"

namespace Quadruped {

qrStateEstimatorContainer::qrStateEstimatorContainer(
    qrRobot *quadrupedIn,
    qrGaitGenerator *gaitGeneratorIn,
    qrUserParameters *userParametersIn,
    std::string terrainConfigPath,
    std::string homeDir)
    :

      quadruped(quadrupedIn),
      gaitGenerator(gaitGeneratorIn),
      userParameters(userParametersIn) {
  groundEstimator = new qrGroundSurfaceEstimator(quadruped, homeDir + terrainConfigPath);

  contactDetection = new qrContactDetection(quadruped, gaitGenerator, groundEstimator);

  robotEstimator = new qrRobotEstimator(quadruped, gaitGenerator, groundEstimator, userParametersIn);

  // _estimators.push_back(groundEsitmator);
  // _estimators.push_back(contactDetection);
  // _estimators.push_back(stateEstimator);
  std::cout << "init state estimator container!" << std::endl;
}

}  // Namespace Quadruped
