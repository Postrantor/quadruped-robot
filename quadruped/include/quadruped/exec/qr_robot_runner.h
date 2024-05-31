/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_RUNNER_H
#define QR_ROBOT_RUNNER_H

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <typeinfo>

#include "action/qr_action.h"
#include "controllers/qr_locomotion_controller.h"
#include "estimators/qr_state_estimator_container.h"
#include "fsm/qr_control_fsm.hpp"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_foothold_planner.h"
#include "planner/qr_pose_planner.h"
#include "robots/qr_robot_a1.h"
#include "robots/qr_robot_a1_sim.h"
#include "robots/qr_robot_go1.h"
#include "robots/qr_robot_lite2_sim.h"
#include "robots/qr_robot_lite3.h"
#include "robots/qr_robot_sim.h"
#include "ros/qr_cmd_vel_receiver.h"
#include "ros/qr_switch_mode_receiver.h"
#include "utils/physics_transform.h"
#include "utils/qr_tools.h"

using namespace Quadruped;

/**
 * @brief 启动所有控制器、规划器和估算器。
 * @param quadruped : 机器人指针。
 * @return LocomotionController 指针。
 */
qrLocomotionController* SetUpController(
    qrRobot* quadruped,
    qrGaitGenerator* gaitGenerator,
    qrDesiredStateCommand* desiredStateCommand,
    qrStateEstimatorContainer* stateEstimators,
    qrUserParameters* userParameters,
    std::string& homeDir);

/**
 * @brief 设置机器人的期望速度。
 */
void UpdateControllerParams(qrLocomotionController* controller, Eigen::Vector3f linSpeed, float angSpeed);

class qrRobotRunner {
public:
  qrRobotRunner(qrRobot* quadruped, std::string& homeDir, ros::NodeHandle& nh);
  ~qrRobotRunner();

  bool Update();
  bool Step();

  inline qrLocomotionController* GetLocomotionController() { return controlFSM->GetLocomotionController(); }
  inline qrStateEstimatorContainer* GetStateEstimator() { return stateEstimators; }
  inline qrDesiredStateCommand* GetDesiredStateCommand() { return desiredStateCommand; }
  inline qrGaitGenerator* GetGaitGenerator() { return gaitGenerator; }

private:
  qrRobot* quadruped;
  qrGaitGenerator* gaitGenerator;
  qrUserParameters userParameters;
  qrStateEstimatorContainer* stateEstimators;
  qrDesiredStateCommand* desiredStateCommand;
  qrControlFSM<float>* controlFSM;

  float resetTime;
  float timeSinceReset;
  std::vector<qrMotorCommand> hybridAction;
};

#endif  // QR_ROBOT_RUNNER_H
