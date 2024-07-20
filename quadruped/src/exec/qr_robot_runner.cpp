/**
 * @brief
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author postrantor
 * @date 2022
 * @copyright MIT License
 */

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "quadruped/exec/qr_robot_runner.h"

float stairsTime = 13;
float stairsVel = 0.1;

qrLocomotionController* SetUpController(
    qrRobot* quadruped,
    qrGaitGenerator* gaitGenerator,
    qrDesiredStateCommand* desiredStateCommand,
    qrStateEstimatorContainer* stateEstimators,
    qrUserParameters* userParameters,
    std::string& homeDir) {
  std::string robot_config_dir = homeDir + "/config/" + quadruped->robotName;

  qrComAdjuster* comAdjuster = new qrComAdjuster(
      quadruped,      //
      gaitGenerator,  //
      stateEstimators->GetRobotEstimator());
  std::cout << "init comAdjuster finish" << std::endl;

  qrPosePlanner* posePlanner = new qrPosePlanner(
      quadruped,      //
      gaitGenerator,  //
      stateEstimators);
  std::cout << "init posePlanner finish" << std::endl;

  qrFootholdPlanner* footholdPlanner = new qrFootholdPlanner(
      quadruped,        //
      gaitGenerator,    //
      stateEstimators,  //
      userParameters,   //
      desiredStateCommand);
  std::cout << "init footholdPlanner finish" << std::endl;

  qrRaibertSwingLegController* swingLegController = new qrRaibertSwingLegController(
      quadruped,        //
      gaitGenerator,    //
      stateEstimators,  //
      footholdPlanner,  //
      *userParameters,  //
      robot_config_dir + "/swing_leg_controller.yaml");
  std::cout << "init swingLegController finish" << std::endl;

  qrStanceLegControllerInterface* stanceLegController = new qrStanceLegControllerInterface(
      quadruped,        //
      gaitGenerator,    //
      stateEstimators,  //
      comAdjuster,      //
      posePlanner,      //
      footholdPlanner,  //
      *userParameters,  //
      robot_config_dir + "/stance_leg_controller.yaml");
  std::cout << "init stanceLegController finish" << std::endl;

  qrLocomotionController* locomotionController = new qrLocomotionController(
      quadruped,            //
      gaitGenerator,        //
      desiredStateCommand,  //
      stateEstimators,      //
      comAdjuster,          //
      posePlanner,          //
      swingLegController,   //
      stanceLegController,  //
      userParameters);
  std::cout << "init locomotionController finish" << std::endl;

  return locomotionController;
}

void UpdateControllerParams(qrLocomotionController* controller, Eigen::Vector3f linSpeed, float angSpeed) {
  controller->swingLegController->desiredSpeed = linSpeed;
  controller->swingLegController->desiredTwistingSpeed = angSpeed;
  controller->stanceLegController->c->desiredSpeed = linSpeed;
  controller->stanceLegController->c->desiredTwistingSpeed = angSpeed;
}

qrRobotRunner::qrRobotRunner(
    qrRobot* quadrupedIn,  //
    std::string& homeDir,  //
    const rclcpp::Node::SharedPtr& nh)
    : quadruped(quadrupedIn),
      desiredStateCommand(new qrDesiredStateCommand(nh, quadruped)),
      userParameters(homeDir + "config/user_parameters.yaml") {
  std::string robot_config_dir = homeDir + "config/" + quadruped->robotName;
  RCLCPP_INFO_STREAM(nh->get_logger(), "[Runner] name: " << quadruped->robotName);

  // main config
  YAML::Node mainConfig = YAML::LoadFile(robot_config_dir + "/main.yaml");
  int twistMode = mainConfig["speed_update_mode"].as<int>();
  std::vector<float> linearVel = mainConfig["const_twist"]["linear"].as<std::vector<float>>();
  Vec3<float> desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
  float desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
  Eigen::MatrixXf::Map(&userParameters.desiredSpeed[0], 3, 1) = desiredSpeed;

  userParameters.desiredTwistingSpeed = desiredTwistingSpeed;
  desiredStateCommand->vDesInBodyFrame = desiredSpeed;
  desiredStateCommand->wDesInBodyFrame << 0, 0, desiredTwistingSpeed;
  quadruped->timeStep = 1.0 / userParameters.controlFrequency;

  quadruped->ReceiveObservation();
  quadruped->ReceiveObservation();
  quadruped->ReceiveObservation();
  Action::StandUp(quadruped, 2.0f, 4.f, 0.001);
  // Action::KeepStand(quadruped, 10,  0.001);
  // Action::ControlFoot(quadruped, nullptr, 15, 0.001);

  if (quadruped->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
    gaitGenerator = new qrWalkGaitGenerator(
        quadruped,  //
        robot_config_dir + "/openloop_gait_generator.yaml");
  } else {
    gaitGenerator = new qrOpenLoopGaitGenerator(
        quadruped,  //
        robot_config_dir + "/openloop_gait_generator.yaml");
  }
  std::cout << "init gaitGenerator finish" << std::endl;

  stateEstimators = new qrStateEstimatorContainer(
      quadruped,        //
      gaitGenerator,    //
      &userParameters,  //
      robot_config_dir + "/terrain.yaml");
  controlFSM = new qrControlFSM<float>(
      quadruped,            //
      stateEstimators,      //
      gaitGenerator,        //
      desiredStateCommand,  //
      &userParameters);
  resetTime = quadruped->GetTimeSinceReset();
  stateEstimators->Reset();
  // gaitGenerator->Reset(resetTime);
  controlFSM->Reset(resetTime);
  stairsVel = userParameters.stairsVel;
  stairsTime = userParameters.stairsTime;
}

bool qrRobotRunner::Update() {
  stateEstimators->Update();
  desiredStateCommand->Update();
  controlFSM->RunFSM(hybridAction);

  return true;
}

bool qrRobotRunner::Step() {
  quadruped->Step(
      qrMotorCommand::convertToMatix(hybridAction),  //
      HYBRID_MODE);

  return 1;
}

qrRobotRunner::~qrRobotRunner() {
  delete quadruped;
  delete gaitGenerator;
  delete stateEstimators;
  delete controlFSM;
}
