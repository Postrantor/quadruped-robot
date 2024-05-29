/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STANCE_LEG_CONTROLLER_INTERFACE_H
#define QR_STANCE_LEG_CONTROLLER_INTERFACE_H

#include "controllers/balance_controller/qr_torque_stance_leg_controller.h"
#include "controllers/mpc/qr_mpc_stance_leg_controller.h"

namespace Quadruped {

/**
 * @brief Interface for the stance leg controller.
 * Use MPC controller or force balance controller.
 */
class qrStanceLegControllerInterface {
public:
  /**
   * @brief constructor of StanceLegControllerInterface.
   * @param robot: pointer to Robot.
   * @param gaitGenerator: pointer to GaitGenerator.
   * @param stateEstimators: pointer to StateEstimatorContainer.
   * @param comAdjuster: pointer to ComAdjuster.
   * @param posePlanner: pointer to PosePlanner.
   * @param footholdPlanner: pointer to FootholdPlanner.
   * @param userParameters: reference of UserParameters.
   * @param configFilepath: the config file path.
   * This is used to determine which stance leg controller to use.
   */
  qrStanceLegControllerInterface(
      qrRobot* robot,
      qrGaitGenerator* gaitGenerator,
      qrStateEstimatorContainer* stateEstimators,
      qrComAdjuster* comAdjuster,
      qrPosePlanner* posePlanner,
      qrFootholdPlanner* footholdPlanner,
      qrUserParameters& userParameters,
      std::string configFilepath);

  /**
   * @brief Destructor of the StanceLegControllerInterface.
   * Delete the force balance controller or MPC controller.
   */
  ~qrStanceLegControllerInterface();

  /**
   * @brief bind desired state command to the stance leg controller that will be used
   * @param pointer to DesiredStateCommand
   */
  void BindCommand(qrDesiredStateCommand* desiredStateCommandIn) { c->BindCommand(desiredStateCommandIn); }

  /**
   * @brief Reset the stance leg controller with current time.
   * @param currentTime: current time to reset.
   */
  void Reset(float currentTime);

  /**
   * @brief Update the stance leg controller with current time every control loop.
   * @param currentTime: current time to update.
   */
  void Update(float currentTime);

  /**
   * @brief get the motor commands of the stance leg controller
   * @return commands and forces calculated by MPC or force balance.
   */
  std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

  /**
   * @brief The pointer to specific stance leg controller.
   */
  TorqueStanceLegController* c;

  /**
   * @brief The pointer to force balance controller.
   */
  TorqueStanceLegController* c1;

  /**
   * @brief The pointer to MPC controller
   */
  TorqueStanceLegController* c2;
};

}  // namespace Quadruped

#endif  // QR_STANCE_LEG_CONTROLLER_INTERFACE_H
