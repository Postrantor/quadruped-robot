/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author GPT4-o
 * @author postrantor
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/utils/qr_utils.hpp"
namespace Quadruped {

/**
 * @brief Construct a new qr Robot A1 Sim::qr Robot A1 Sim object
 * @details 读取配置文件，初始化参数，构造机器人对象
 * @param nh no use
 * @param configFilePath
 */
qrRobotA1Sim::qrRobotA1Sim(const rclcpp::Node::SharedPtr& nh, std::string configFilePath) : nhIn(nh) {
  /// read paramsters
  // 初始化变量
  baseOrientation << 1.f, 0.f, 0.f, 0.f;
  baseRollPitchYaw << 0.f, 0.f, 0.f;
  baseRollPitchYawRate << 0.f, 0.f, 0.f;
  footForce << 0.f, 0.f, 0.f, 0.f;
  footContact << 1, 1, 1, 1;
  motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();

  // 读取配置文件
  this->configFilePath = configFilePath;
  robotConfig = YAML::LoadFile(configFilePath);

  robotName = robotConfig["name"].as<std::string>();
  isSim = robotConfig["is_sim"].as<bool>();

  // 重量
  totalMass = robotConfig["robot_params"]["total_mass"].as<float>();
  bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();
  // 惯性
  std::vector<float> totalInertiaVec = robotConfig["robot_params"]["total_inertia"].as<std::vector<float>>();
  totalInertia = Eigen::MatrixXf::Map(&totalInertiaVec[0], 3, 3);
  std::vector<float> bodyInertiaVec = robotConfig["robot_params"]["body_inertia"].as<std::vector<float>>();
  bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaVec[0], 3, 3);
  std::vector<std::vector<float>> inertias =
      robotConfig["robot_params"]["links_inertia"].as<std::vector<std::vector<float>>>();
  std::vector<float> masses = robotConfig["robot_params"]["links_mass"].as<std::vector<float>>();
  std::vector<std::vector<float>> linksComPos_ =
      robotConfig["robot_params"]["links_com_pos"].as<std::vector<std::vector<float>>>();

  std::string inertias_str = array_to_string(inertias[0], "inertias: ");
  RCLCPP_INFO_STREAM(nhIn->get_logger(), "\n\t" << inertias_str);

  for (int legId = 0; legId < NumLeg; ++legId) {
    Mat3<float> inertia = Mat3<float>::Zero();
    inertia = Eigen::MatrixXf::Map(&inertias[0][0], 3, 3);
    linkInertias.push_back(inertia);  // hip link
    inertia = Eigen::MatrixXf::Map(&inertias[1][0], 3, 3);
    linkInertias.push_back(inertia);  // thigh link
    inertia = Eigen::MatrixXf::Map(&inertias[2][0], 3, 3);
    linkInertias.push_back(inertia);  // calf link

    linkMasses.push_back(masses[0]);
    linkMasses.push_back(masses[1]);
    linkMasses.push_back(masses[2]);

    linksComPos.push_back(linksComPos_[0]);
    linksComPos.push_back(linksComPos_[1]);
    linksComPos.push_back(linksComPos_[2]);
  }

  bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();
  std::vector<float> abadLocation_ = robotConfig["robot_params"]["abad_location"].as<std::vector<float>>();
  abadLocation = Eigen::MatrixXf::Map(&abadLocation_[0], 3, 1);
  hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
  upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
  lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();

  std::vector<std::vector<float>> defaultHipPositionList =
      robotConfig["robot_params"]["default_hip_positions"].as<std::vector<std::vector<float>>>();
  Eigen::Matrix<float, 3, 1> defaultHipPositionFR = Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionFL = Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionRL = Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionRR = Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
  defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;

  float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
  sitDownAbAngle = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
  sitDownHipAngle = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
  sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
  Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
  sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

  // read `motor_params`
  float abadKp, abadKd, hipKp, hipKd, kneeKp, kneeKd;
  abadKp = robotConfig["motor_params"]["abad_p"].as<float>();
  abadKd = robotConfig["motor_params"]["abad_d"].as<float>();
  hipKp = robotConfig["motor_params"]["hip_p"].as<float>();
  hipKd = robotConfig["motor_params"]["hip_d"].as<float>();
  kneeKp = robotConfig["motor_params"]["knee_p"].as<float>();
  kneeKd = robotConfig["motor_params"]["knee_d"].as<float>();
  Eigen::Matrix<float, 3, 1> kps(abadKp, hipKp, kneeKp);
  Eigen::Matrix<float, 3, 1> kds(abadKd, hipKd, kneeKd);
  motorKps << kps, kps, kps, kps;
  motorKds << kds, kds, kds, kds;

  std::vector<float> jointDirectionList = robotConfig["motor_params"]["joint_directions"].as<std::vector<float>>();
  std::vector<float> jointOffsetList = robotConfig["motor_params"]["joint_offsets"].as<std::vector<float>>();
  jointDirection = Eigen::MatrixXf::Map(&jointDirectionList[0], 12, 1);
  jointOffset = Eigen::MatrixXf::Map(&jointOffsetList[0], 12, 1);

  float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
  standUpAbAngle = 0.f;
  standUpHipAngle = std::acos(bodyHeight / 2.f / upperLegLength);
  standUpKneeAngle = -2.f * standUpHipAngle;
  Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
  standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

  controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();

  /// reset com_offset
  Reset();

  /// callback
  imuSub = nhIn->create_subscription<sensor_msgs::msg::Imu>(
      "a1_gazebo/trunk_imu", 1, std::bind(&qrRobotA1Sim::ImuCallback, this, std::placeholders::_1));

  jointStateSub[0] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FR_hip_controller/state", 1, std::bind(&qrRobotA1Sim::FRhipCallback, this, std::placeholders::_1));
  jointStateSub[1] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FR_thigh_controller/state", 1, std::bind(&qrRobotA1Sim::FRthighCallback, this, std::placeholders::_1));
  jointStateSub[2] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FR_calf_controller/state", 1, std::bind(&qrRobotA1Sim::FRcalfCallback, this, std::placeholders::_1));
  jointStateSub[3] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FL_hip_controller/state", 1, std::bind(&qrRobotA1Sim::FLhipCallback, this, std::placeholders::_1));
  jointStateSub[4] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FL_thigh_controller/state", 1, std::bind(&qrRobotA1Sim::FLthighCallback, this, std::placeholders::_1));
  jointStateSub[5] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/FL_calf_controller/state", 1, std::bind(&qrRobotA1Sim::FLcalfCallback, this, std::placeholders::_1));
  jointStateSub[6] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RR_hip_controller/state", 1, std::bind(&qrRobotA1Sim::RRhipCallback, this, std::placeholders::_1));
  jointStateSub[7] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RR_thigh_controller/state", 1, std::bind(&qrRobotA1Sim::RRthighCallback, this, std::placeholders::_1));
  jointStateSub[8] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RR_calf_controller/state", 1, std::bind(&qrRobotA1Sim::RRcalfCallback, this, std::placeholders::_1));
  jointStateSub[9] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RL_hip_controller/state", 1, std::bind(&qrRobotA1Sim::RLhipCallback, this, std::placeholders::_1));
  jointStateSub[10] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RL_thigh_controller/state", 1, std::bind(&qrRobotA1Sim::RLthighCallback, this, std::placeholders::_1));
  jointStateSub[11] = nhIn->create_subscription<unitree_msgs::msg::MotorState>(
      "a1_gazebo/RL_calf_controller/state", 1, std::bind(&qrRobotA1Sim::RLcalfCallback, this, std::placeholders::_1));

  jointCmdPub[0] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FR_hip_controller/command", 1);
  jointCmdPub[1] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FR_thigh_controller/command", 1);
  jointCmdPub[2] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FR_calf_controller/command", 1);
  jointCmdPub[3] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FL_hip_controller/command", 1);
  jointCmdPub[4] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FL_thigh_controller/command", 1);
  jointCmdPub[5] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/FL_calf_controller/command", 1);
  jointCmdPub[6] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RR_hip_controller/command", 1);
  jointCmdPub[7] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RR_thigh_controller/command", 1);
  jointCmdPub[8] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RR_calf_controller/command", 1);
  jointCmdPub[9] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RL_hip_controller/command", 1);
  jointCmdPub[10] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RL_thigh_controller/command", 1);
  jointCmdPub[11] = nhIn->create_publisher<unitree_msgs::msg::MotorCmd>("a1_gazebo/RL_calf_controller/command", 1);

  // footForceSub[0] = nhIn->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     "/visual/FR_foot_contact/the_force", 1, std::bind(&qrRobotA1Sim::FRfootCallback, this, std::placeholders::_1));
  // footForceSub[1] = nhIn->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     "/visual/FL_foot_contact/the_force", 1, std::bind(&qrRobotA1Sim::FLfootCallback, this, std::placeholders::_1));
  // footForceSub[2] = nhIn->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     "/visual/RR_foot_contact/the_force", 1, std::bind(&qrRobotA1Sim::RRfootCallback, this, std::placeholders::_1));
  // footForceSub[3] = nhIn->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     "/visual/RL_foot_contact/the_force", 1, std::bind(&qrRobotA1Sim::RLfootCallback, this, std::placeholders::_1));

  // ros::spinOnce();
  // must wait 300ms, to get first state
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  yawOffset = 0;  // lowState.imu.rpy[2]; // todo
  RCLCPP_INFO_STREAM(
      nhIn->get_logger(), "\n\t"
                              << "yaw offset: " << yawOffset);

  // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
  this->ResetTimer();
  lastResetTime = GetTimeSinceReset();
  initComplete = true;

  RCLCPP_INFO_STREAM(nhIn->get_logger(), "sim robot object init complete.");
}

bool qrRobotA1Sim::BuildDynamicModel() {
  // we assume the cheetah's body (not including rotors) can be modeled as a
  // uniformly distributed box.
  std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>();
  Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);  // Length, Width, Height

  // locations
  Vec3<float> _abadRotorLocation = {0.14f, 0.047f, 0.f};
  Vec3<float> _abadLocation = {0.1805f, 0.047f, 0.f};
  Vec3<float> _hipLocation = Vec3<float>(0, hipLength, 0);
  Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.04, 0);
  Vec3<float> _kneeLocation = Vec3<float>(0, 0, -upperLegLength);
  Vec3<float> _kneeRotorLocation = Vec3<float>(0, 0, 0);

  float scale_ = 1e-2;
  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<float> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ.setIdentity();
  rotorRotationalInertiaZ = scale_ * 1e-6 * rotorRotationalInertiaZ;

  Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
  Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
  Mat3<float> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<float> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<float> abadRotationalInertia;
  abadRotationalInertia << 469.2, -9.4, -0.342, -9.4, 807.5, -0.466, -0.342, -0.466, 552.9;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<float> abadCOM(-0.0033, 0, 0);
  SpatialInertia<float> abadInertia(0.696, abadCOM, abadRotationalInertia);

  Mat3<float> hipRotationalInertia;
  hipRotationalInertia << 5529, 4.825, 343.9, 4.825, 5139.3, 22.4, 343.9, 22.4, 1367.8;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<float> hipCOM(-0.003237, -0.022327, -0.027326);  // left, for right filp y-axis value.
  SpatialInertia<float> hipInertia(1.013, hipCOM, hipRotationalInertia);
  RCLCPP_INFO_STREAM(
      nhIn->get_logger(), "hip inertia: \n"                     //
                              << hipInertia.getInertiaTensor()  //
                              << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor());

  Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 2998, 0, -141.2, 0, 3014, 0, -141.2, 0, 32.4;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = kneeRotationalInertiaRotated;  // RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<float> kneeCOM(0.006435, 0, -0.107);
  SpatialInertia<float> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

  Vec3<float> rotorCOM(0, 0, 0);
  float rotorMass = 1e-8;  // 0.055
  SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

  Mat3<float> bodyRotationalInertia;
  bodyRotationalInertia << 15853, 0, 0, 0, 37799, 0, 0, 0, 45654;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<float> bodyCOM(0, 0, 0);
  SpatialInertia<float> bodyInertia(6, bodyCOM, bodyRotationalInertia);

  model.addBase(bodyInertia);
  // add contact for the cheetah's body
  model.addGroundContactBoxPoints(5, bodyDims);

  const int baseID = 5;
  int bodyID = baseID;
  float sideSign = -1;

  Mat3<float> I3 = Mat3<float>::Identity();

  auto& abadRotorInertia = rotorInertiaX;
  float abadGearRatio = 1;  // 6
  auto& hipRotorInertia = rotorInertiaY;
  float hipGearRatio = 1;  // 6
  auto& kneeRotorInertia = rotorInertiaY;
  float kneeGearRatio = 1;  // 9.33
  float kneeLinkY_offset = 0.004;

  // loop over 4 legs
  for (int legID = 0; legID < 4; legID++) {
    bodyID++;
    Mat6<float> xtreeAbad = createSXform(
        I3,  //
        WithLegSigns(_abadLocation, legID));
    Mat6<float> xtreeAbadRotor = createSXform(
        I3,  //
        WithLegSigns(_abadRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(
          abadInertia.flipAlongAxis(CoordinateAxis::Y),       //
          abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),  //
          abadGearRatio,                                      //
          baseID,                                             //
          JointType::Revolute,                                //
          CoordinateAxis::X,                                  //
          xtreeAbad,                                          //
          xtreeAbadRotor);
    } else {
      model.addBody(
          abadInertia,          //
          abadRotorInertia,     //
          abadGearRatio,        //
          baseID,               //
          JointType::Revolute,  //
          CoordinateAxis::X,    //
          xtreeAbad,            //
          xtreeAbadRotor);
    }

    // Hip Joint
    bodyID++;
    Mat6<float> xtreeHip = createSXform(
        I3,                                  // coordinateRotation(CoordinateAxis::Z, float(M_PI)),
        WithLegSigns(_hipLocation, legID));  // 0, hipLength=0.085, 0
    Mat6<float> xtreeHipRotor = createSXform(
        coordinateRotation(
            CoordinateAxis::Z,  //
            float(M_PI)),       //
        WithLegSigns(_hipRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(
          hipInertia.flipAlongAxis(CoordinateAxis::Y),       //
          hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),  //
          hipGearRatio,                                      //
          bodyID - 1,                                        //
          JointType::Revolute,                               //
          CoordinateAxis::Y,                                 //
          xtreeHip,                                          //
          xtreeHipRotor);
    } else {
      model.addBody(
          hipInertia,                   //
          hipRotorInertia,              //
          hipGearRatio,                 //
          bodyID - 1,                   //
          JointType::Revolute,          //
          CoordinateAxis::Y, xtreeHip,  //
          xtreeHipRotor);
    }

    // add knee ground contact point
    model.addGroundContactPoint(
        bodyID,  //
        Vec3<float>(0, 0, -upperLegLength));

    // Knee Joint
    bodyID++;
    Mat6<float> xtreeKnee = createSXform(
        I3,  //
        _kneeLocation);
    Mat6<float> xtreeKneeRotor = createSXform(
        I3,  //
        _kneeRotorLocation);
    if (sideSign < 0) {
      model.addBody(
          kneeInertia,                                        //.flipAlongAxis(CoordinateAxis::Y),
          kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),  //
          kneeGearRatio,                                      //
          bodyID - 1,                                         //
          JointType::Revolute,                                //
          CoordinateAxis::Y,                                  //
          xtreeKnee,                                          //
          xtreeKneeRotor);
      model.addGroundContactPoint(
          bodyID,                                             //
          Vec3<float>(0, kneeLinkY_offset, -lowerLegLength),  //
          true);
    } else {
      model.addBody(
          kneeInertia,          //
          kneeRotorInertia,     //
          kneeGearRatio,        //
          bodyID - 1,           //
          JointType::Revolute,  //
          CoordinateAxis::Y,    //
          xtreeKnee,            //
          xtreeKneeRotor);
      model.addGroundContactPoint(
          bodyID,                                              //
          Vec3<float>(0, -kneeLinkY_offset, -lowerLegLength),  //
          true);
    }

    // add foot
    // model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

    sideSign *= -1;
  }

  Vec3<float> g(0, 0, -9.81);
  model.setGravity(g);

  bool test_fb = false;
  if (test_fb) {
    FBModelState<float> fb;
    // for (size_t i(0); i < 3; ++i) {
    //   _state.bodyVelocity[i] = omegaBody[i];  // in body frame
    //   _state.bodyVelocity[i + 3] = vBody[i];

    //   for (size_t leg(0); leg < 4; ++leg) {
    //     _state.q[3 * leg + i] = q[3 * leg + i];
    //     _state.qd[3 * leg + i] = dq[3 * leg + i];
    //     _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    //   }
    // }
    RCLCPP_INFO_STREAM(nhIn->get_logger(), "339");

    fb.bodyOrientation << 1, 0, 0, 0;  // 0.896127, 0.365452,0.246447,-0.0516205;
    // fb.bodyPosition << 0.00437649, 0.000217693, 0.285963;
    fb.bodyPosition.setZero();
    fb.bodyVelocity << 3, 3, 3, 0.2, 0.1, 0.1;
    RCLCPP_INFO_STREAM(nhIn->get_logger(), "343");

    fb.q.resize(12, 1);
    fb.q.setZero();
    fb.q << 0.2, 0, -0.2,  //
        0.2, 0, -0.2,      // 0, 0.7, 0,
        0, 0.3, 0.5,       // 0, 0.8, 0
        0, 0.3, 0.5,       //
        fb.qd.resize(12, 1);
    fb.qd.setZero();
    // fb.qd << 0.2, 0, 0, 0.1, 0, 0., 0, -0.3, 0.6, 0, 0.3, 1;
    RCLCPP_INFO_STREAM(nhIn->get_logger(), "346");

    model.setState(fb);
    RCLCPP_INFO_STREAM(nhIn->get_logger(), "348");

    model.forwardKinematics();
    model.massMatrix();
    Eigen::MatrixXf A;
    Eigen::Matrix<float, 18, 1> dq;
    dq << fb.bodyVelocity, fb.qd;
    // model.generalizedGravityForce();
    // model.generalizedCoriolisForce();
    A = model.getMassMatrix();
    for (int i = 0; i < 18; ++i) {
      for (int j = 0; j < 18; ++j) {
        if (A(i, j) < 1e-6) {
          A(i, j) = 0;
        }
      }
    }
    float energy = 0.5 * dq.dot(A * dq);
    std::cout << "A = \n" << A << std::endl;
    std::cout << "energy = " << energy << std::endl;
    RCLCPP_INFO_STREAM(
        nhIn->get_logger(), "\n\t"
                                << "A = " << A << "\n\t"
                                << "energy" << energy);

    // model.getPosition(8);
    RCLCPP_INFO_STREAM(nhIn->get_logger(), "351");

    // FIXME(zhiqi.jia) :: why use throw?
    throw std::domain_error("finished!!!!");
  }
  return true;
}

void qrRobotA1Sim::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // set quaternion
  lowState.imu.quaternion[0] = msg->orientation.w;
  lowState.imu.quaternion[1] = msg->orientation.x;
  lowState.imu.quaternion[2] = msg->orientation.y;
  lowState.imu.quaternion[3] = msg->orientation.z;

  // convet to RPY
  Eigen::Matrix<float, 4, 1> quaternion = {
      lowState.imu.quaternion[0],  //
      lowState.imu.quaternion[1],  //
      lowState.imu.quaternion[2],  //
      lowState.imu.quaternion[3]};
  Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);
  lowState.imu.rpy[0] = rpy[0];
  lowState.imu.rpy[1] = rpy[1];
  lowState.imu.rpy[2] = rpy[2];

  // set gyroscope
  lowState.imu.gyroscope[0] = msg->angular_velocity.x;
  lowState.imu.gyroscope[1] = msg->angular_velocity.y;
  lowState.imu.gyroscope[2] = msg->angular_velocity.z;

  // set accelerometer
  lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
  lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
  lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
}

void qrRobotA1Sim::MotorStateCallback(int index, const unitree_msgs::msg::MotorState::SharedPtr msg) {
  lowState.motor_state[index].mode = msg->mode;
  lowState.motor_state[index].q = msg->q;
  lowState.motor_state[index].dq = msg->dq;
  lowState.motor_state[index].tau = msg->tau;
  lowState.motor_state[index].temp = msg->temp;
}
void qrRobotA1Sim::FRhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(0, msg); }
void qrRobotA1Sim::FRthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(1, msg); }
void qrRobotA1Sim::FRcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(2, msg); }
void qrRobotA1Sim::FLhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(3, msg); }
void qrRobotA1Sim::FLthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(4, msg); }
void qrRobotA1Sim::FLcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(5, msg); }
void qrRobotA1Sim::RRhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(6, msg); }
void qrRobotA1Sim::RRthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(7, msg); }
void qrRobotA1Sim::RRcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(8, msg); }
void qrRobotA1Sim::RLhipCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(9, msg); }
void qrRobotA1Sim::RLthighCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(10, msg); }
void qrRobotA1Sim::RLcalfCallback(const unitree_msgs::msg::MotorState::SharedPtr msg) { MotorStateCallback(11, msg); }

// void qrRobotA1Sim::FootCallback(int index, const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
//   lowState.foot_force_est[index].x = msg->wrench.force.x;
//   lowState.foot_force_est[index].y = msg->wrench.force.y;
//   lowState.foot_force_est[index].z = msg->wrench.force.z;
//   lowState.foot_force[index] = msg->wrench.force.z;
// }
// void qrRobotA1Sim::FRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { FootCallback(0, msg); }
// void qrRobotA1Sim::FLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { FootCallback(1, msg); }
// void qrRobotA1Sim::RRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { FootCallback(2, msg); }
// void qrRobotA1Sim::RLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { FootCallback(3, msg); }

// FIXME(zhiqi.jia) :: why use motor_id * 5?
void qrRobotA1Sim::SendCommand(const std::array<float, 60> motor_cmd) {
  for (int id = 0; id < 12; id++) {
    lowCmd.motor_cmd[id].mode = 0x0A;
    lowCmd.motor_cmd[id].q = motor_cmd[id * 5];
    lowCmd.motor_cmd[id].dq = motor_cmd[id * 5 + 2];
    lowCmd.motor_cmd[id].k_q = motor_cmd[id * 5 + 1];
    lowCmd.motor_cmd[id].k_dq = motor_cmd[id * 5 + 3];
    lowCmd.motor_cmd[id].tau = motor_cmd[id * 5 + 4];

    jointCmdPub[id]->publish(lowCmd.motor_cmd[id]);
  }
}

void qrRobotA1Sim::ReceiveObservation() {
  baseAccInBaseFrame << lowState.imu.accelerometer[0], lowState.imu.accelerometer[1], lowState.imu.accelerometer[2];

  stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);

  // calibrated
  float calibratedYaw = lowState.imu.rpy[2] - yawOffset;
  if (calibratedYaw >= M_PI) {
    calibratedYaw -= M_2PI;
  } else if (calibratedYaw <= -M_PI) {
    calibratedYaw += M_2PI;
  }
  if (abs(baseRollPitchYaw[2] - calibratedYaw) >= M_PI) {
    rpyFilter.Reset();
  }
  Vec3<float> rpyVec(lowState.imu.rpy[0], lowState.imu.rpy[1], calibratedYaw);
  baseRollPitchYaw = rpyFilter.CalculateAverage(rpyVec);
  if (baseRollPitchYaw[2] >= M_PI) {
    baseRollPitchYaw[2] -= M_2PI;
  } else if (baseRollPitchYaw[2] <= -M_PI) {
    baseRollPitchYaw[2] += M_2PI;
  }

  Vec3<float> gyroVec(lowState.imu.gyroscope[0], lowState.imu.gyroscope[1], lowState.imu.gyroscope[2]);
  baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);

  baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

  for (int motor_id = 0; motor_id < NumMotor; ++motor_id) {
    motorAngles[motor_id] = lowState.motor_state[motor_id].q;
    motorVelocities[motor_id] = lowState.motor_state[motor_id].dq;
    // FIXME(zhiqi.jia) :: error assign
    motorddq[motor_id] = lowState.motor_state[motor_id].temp;
    motortorque[motor_id] = lowState.motor_state[motor_id].tau;
  }

  // boost::is_array<int16_t, 4> force = lowState.foot_force;
  // footForce << force[0], force[1], force[2], force[3];
  // for (int footId = 0; footId < NumLeg; footId++) {
  //   if (footForce[footId] >= 5) {
  //     footContact[footId] = true;
  //   } else {
  //     footContact[footId] = false;
  //   }
  // }

  UpdateDataFlow();
}

void qrRobotA1Sim::ApplyAction(const Eigen::MatrixXf& motorCommands, MotorMode motorControlMode) {
  std::array<float, 60> motorCommandsArray = {0};
  if (motorControlMode == POSITION_MODE) {
    Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
    for (int motor_id = 0; motor_id < NumMotor; motor_id++) {
      motorCommandsArray[motor_id * 5] = motorCommandsShaped[motor_id];
      motorCommandsArray[motor_id * 5 + 1] = motorKps[motor_id];
      motorCommandsArray[motor_id * 5 + 2] = 0;
      motorCommandsArray[motor_id * 5 + 3] = motorKds[motor_id];
      motorCommandsArray[motor_id * 5 + 4] = 0;
    }
  } else if (motorControlMode == TORQUE_MODE) {
    Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
    for (int motor_id = 0; motor_id < NumMotor; motor_id++) {
      motorCommandsArray[motor_id * 5] = 0;
      motorCommandsArray[motor_id * 5 + 1] = 0;
      motorCommandsArray[motor_id * 5 + 2] = 0;
      motorCommandsArray[motor_id * 5 + 3] = 0;
      motorCommandsArray[motor_id * 5 + 4] = motorCommandsShaped[motor_id];
    }
  } else if (motorControlMode == HYBRID_MODE) {
    Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
    for (int motor_id = 0; motor_id < NumMotor; ++motor_id) {
      motorCommandsArray[motor_id * 5] = motorCommandsShaped(POSITION, motor_id);
      motorCommandsArray[motor_id * 5 + 1] = motorCommandsShaped(KP, motor_id);
      motorCommandsArray[motor_id * 5 + 2] = motorCommandsShaped(VELOCITY, motor_id);
      motorCommandsArray[motor_id * 5 + 3] = motorCommandsShaped(KD, motor_id);
      motorCommandsArray[motor_id * 5 + 4] = motorCommandsShaped(TORQUE, motor_id);
    }
  }

  for (int index = 0; index < motorCommandsArray.size(); index++) {
    if (isnan(motorCommandsArray[index])) {
      motorCommandsArray[index] = 0.f;
    }
  }

  SendCommand(motorCommandsArray);
}

// for real robot
void qrRobotA1Sim::ApplyAction(const std::vector<qrMotorCommand>& motorCommands, MotorMode motorControlMode) {
  std::array<float, 60> motorCommandsArray = {0};
  for (int motor_id = 0; motor_id < NumMotor; motor_id++) {
    motorCommandsArray[motor_id * 5] = motorCommands[motor_id].p;
    motorCommandsArray[motor_id * 5 + 1] = motorCommands[motor_id].Kp;
    motorCommandsArray[motor_id * 5 + 2] = motorCommands[motor_id].d;
    motorCommandsArray[motor_id * 5 + 3] = motorCommands[motor_id].Kd;
    motorCommandsArray[motor_id * 5 + 4] = motorCommands[motor_id].tua;
  }

  // send to real robot, use unitree sdk
  // robotInterface.SendCommand(motorCommandsArray);
}

void qrRobotA1Sim::Step(const Eigen::MatrixXf& action, MotorMode motorControlMode) {
  ReceiveObservation();
  ApplyAction(action, motorControlMode);
}

}  // namespace Quadruped
