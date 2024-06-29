/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "controllers/qr_stance_leg_controller_interface.h"

namespace Quadruped {

qrStanceLegControllerInterface::qrStanceLegControllerInterface(
    qrRobot *robot,
    qrGaitGenerator *gait_generator,
    qrStateEstimatorContainer *state_estimators,
    qrComAdjuster *com_adjuster,
    qrPosePlanner *pose_planner,
    qrFootholdPlanner *foothold_planner,
    qrUserParameters &user_parameters,
    std::string config_filepath) {
  c = nullptr;
  c1 = new TorqueStanceLegController(
      robot, gait_generator, state_estimators, com_adjuster, pose_planner, foothold_planner, user_parameters,
      config_filepath);

  c2 = new MPCStanceLegController(
      robot, gait_generator, state_estimators, com_adjuster, pose_planner, foothold_planner, user_parameters,
      config_filepath);

  if (robot->controlParams["mode"] == LocomotionMode::ADVANCED_TROT) {
    c = c2;
  } else {
    std::cout << "[STANCE CONTROLLER INTERFACE] use TorqueStanceLegController" << std::endl;
    // TorqueStanceLegController, TorqueStanceLegControllerMPC
    c = c1;
  }
}

qrStanceLegControllerInterface::~qrStanceLegControllerInterface() {
  if (c1) {
    delete c1;
  }
  if (c2) {
    delete c2;
  }
  c = nullptr;
  c1 = nullptr;
  c2 = nullptr;
}

void qrStanceLegControllerInterface::Reset(float current_time) {
  if (c->robot->controlParams["mode"] != LocomotionMode::ADVANCED_TROT) {
    c = c1;
  } else {
    c = static_cast<MPCStanceLegController *>(c2);
  }
  c->Reset(current_time);
}

void qrStanceLegControllerInterface::Update(float current_time) { c->Update(current_time); }

std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrStanceLegControllerInterface::GetAction() {
  return c->GetAction();
}

}  // namespace Quadruped
