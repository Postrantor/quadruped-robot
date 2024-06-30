/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef NOT_USE_SIM

#include "ros/qr_control2gazebo_msg.h"

namespace Quadruped {

qrController2GazeboMsg::qrController2GazeboMsg(
    qrRobot* robotIn,  //
    qrLocomotionController* locomotionControllerIn)
    : Node("qr_controller2gazebo_msg"), robot(robotIn), locomotionController(locomotionControllerIn) {
  posePlanner = locomotionController->GetPosePlanner();
  gazeboStatePublish = this->create_publisher<xpp_msgs::msg::RobotStateCartesian>(xpp_msgs::robot_state_current, 10);
  gazeboParamPublish = this->create_publisher<xpp_msgs::msg::RobotParameters>(xpp_msgs::robot_parameters, 10);
  posePlannerPublish = this->create_publisher<geometry_msgs::msg::Pose>(xpp_msgs::robot_state_desired, 10);
  if (robot->isSim) {
    baseStateClient = this->create_client<gazebo_msgs::srv::GetLinkState>("/gazebo/get_link_state");
  }

  desiredPoseFramebr = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  lastTime = this->now();
  RCLCPP_INFO(this->get_logger(), "Controller2GazeboMsg init success...");
}

void qrController2GazeboMsg::PublishGazeboStateCallback() {
  auto curTime = this->now();

  const auto& q = robot->GetBaseOrientation();
  const auto& pose = robot->GetBasePosition();
  Vec3<float> wInBodyFrame = robot->GetBaseRollPitchYawRate();
  const auto bodyVInWorldFrame = robot->stateDataFlow.baseVInWorldFrame;
  const auto& jointAngles = robot->GetMotorAngles();
  const auto& forces = robot->GetFootForce();
  auto contacts = robot->GetFootContact();
  auto& legState = locomotionController->GetGaitGenerator()->legState;
  for (int legId(0); legId < 4; ++legId) {
    contacts[legId] = !(legState[legId] == LegState::SWING);
  }
  const auto& footPos = robot->GetFootPositionsInWorldFrame();

  xpp::RobotStateCartesian robotstate(4);

  robotstate.t_global_ = robot->GetTimeSinceReset();
  if (!robot->isSim) {
    robotstate.base_.lin.p_.x() = pose[0];
    robotstate.base_.lin.p_.y() = pose[1];
    robotstate.base_.lin.p_.z() = pose[2];

    robotstate.base_.lin.v_.x() = bodyVInWorldFrame[0];
    robotstate.base_.lin.v_.y() = bodyVInWorldFrame[1];
    robotstate.base_.lin.v_.z() = bodyVInWorldFrame[2];

    robotstate.base_.ang.q = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    robotstate.base_.ang.w = wInBodyFrame.cast<double>();

  } else {
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->link_name = "a1_gazebo::base";
    request->reference_frame = "world";
    if (baseStateClient->wait_for_service(std::chrono::seconds(1))) {
      auto result = baseStateClient->async_send_request(request);
      if (result.get()->success) {
        const auto& pose_ = result.get()->link_state.pose;
        robotstate.base_.lin.p_.x() = pose_.position.x;
        robotstate.base_.lin.p_.y() = pose_.position.y;
        robotstate.base_.lin.p_.z() = pose_.position.z;
        robotstate.base_.ang.q =
            Eigen::Quaterniond(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
      } else {
        RCLCPP_INFO(this->get_logger(), "Get Gazebo link state not success!");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Get Gazebo link state service not available!");
    }
  }

  auto& joint_states = robotstate.joint_states;
  const Eigen::Matrix<float, 3, 4>& footholds =
      locomotionController->GetSwingLegController()->footTargetPositionsInWorldFrame;
  for (int legId = 0; legId < 4; ++legId) {
    robotstate.ee_motion_.at(legId).p_.x() = footPos(0, legId);
    robotstate.ee_motion_.at(legId).p_.y() = footPos(1, legId);
    robotstate.ee_motion_.at(legId).p_.z() = footPos(2, legId);
    robotstate.ee_foothold_.at(legId).x() = footholds(0, legId);
    robotstate.ee_foothold_.at(legId).y() = footholds(1, legId);
    robotstate.ee_foothold_.at(legId).z() = footholds(2, legId);
    robotstate.ee_forces_.at(legId).x() = 0;
    robotstate.ee_forces_.at(legId).y() = 0;
    robotstate.ee_forces_.at(legId).z() = forces[legId];
    robotstate.ee_contact_.at(legId) = contacts[legId];
    for (int j = 0; j < 3; ++j) {
      joint_states.at(3 * legId + j) = double(jointAngles[3 * legId + j]);
    }
  }

  xpp_msgs::msg::RobotParameters workspace;
  workspace.base_mass = 10.0;
  workspace.ee_names = {"RF"};
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 0;
  p.z = -1;
  workspace.nominal_ee_pos = {p};

  if ((rIB - posePlanner->rIB).norm() > 1e-3) {
    rIB = posePlanner->rIB;
    quat = posePlanner->quat;
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = rIB[0];
    poseMsg.position.y = rIB[1];
    poseMsg.position.z = rIB[2];
    poseMsg.orientation.w = quat[0];
    poseMsg.orientation.x = quat[1];
    poseMsg.orientation.y = quat[2];
    poseMsg.orientation.z = quat[3];

    posePlannerPublish->publish(poseMsg);
  }
  gazeboParamPublish->publish(workspace);
  gazeboStatePublish->publish(xpp::Convert::ToRos(robotstate));

  transform.translation.x = rIB[0];
  transform.translation.y = rIB[1];
  transform.translation.z = rIB[2];
  transform.rotation.w = quat[0];
  transform.rotation.x = quat[1];
  transform.rotation.y = quat[2];
  transform.rotation.z = quat[3];
  desiredPoseFramebr->sendTransform(
      tf2::Stamped<geometry_msgs::msg::Transform>(transform, this->now(), "world", "desiredPoseFrame"));
}

}  // namespace Quadruped
