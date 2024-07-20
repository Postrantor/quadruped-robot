/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_RUNNER_H
#define QR_ROBOT_RUNNER_H

#include <iostream>
#include <typeinfo>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "quadruped/action/qr_action.h"
#include "quadruped/controllers/qr_locomotion_controller.h"
#include "quadruped/estimators/qr_state_estimator_container.h"
#include "quadruped/fsm/qr_control_fsm.hpp"
#include "quadruped/planner/qr_com_adjuster.h"
#include "quadruped/planner/qr_foothold_planner.h"
#include "quadruped/planner/qr_pose_planner.h"
#include "quadruped/robots/qr_robot_a1.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_cmd_vel_receiver.h"
#include "quadruped/ros/qr_switch_mode_receiver.h"
#include "quadruped/utils/physics_transform.h"
#include "quadruped/utils/qr_tools.h"

using namespace Quadruped;

/**
 * @brief 启动所有控制器、规划器和估算器
 * @param quadruped : 机器人指针
 * @return LocomotionController 指针
 */
qrLocomotionController *SetUpController(
    qrRobot *quadruped,
    qrGaitGenerator *gaitGenerator,
    qrDesiredStateCommand *desiredStateCommand,
    qrStateEstimatorContainer *stateEstimators,
    qrUserParameters *userParameters,
    std::string &homeDir);

/**
 * @brief 设置机器人的期望速度
 */
void UpdateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

class qrRobotRunner {
public:
  qrRobotRunner(qrRobot *quadruped, std::string &homeDir, const rclcpp::Node::SharedPtr &nh);
  ~qrRobotRunner();

  bool Update();
  bool Step();

  inline qrLocomotionController *GetLocomotionController() { return controlFSM->GetLocomotionController(); }
  inline qrStateEstimatorContainer *GetStateEstimator() { return stateEstimators; }
  inline qrDesiredStateCommand *GetDesiredStateCommand() { return desiredStateCommand; }
  inline qrGaitGenerator *GetGaitGenerator() { return gaitGenerator; }

private:
  qrRobot *quadruped;
  qrGaitGenerator *gaitGenerator;
  qrUserParameters userParameters;
  qrStateEstimatorContainer *stateEstimators;
  qrDesiredStateCommand *desiredStateCommand;
  qrControlFSM<float> *controlFSM;

  float resetTime;
  float timeSinceReset;
  std::vector<qrMotorCommand> hybridAction;
};

#endif  // QR_ROBOT_RUNNER_H
