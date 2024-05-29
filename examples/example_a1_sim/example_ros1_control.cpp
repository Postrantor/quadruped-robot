/**
 * @author GPT4-o
 * @brief
 * @date 2024-05-29
 * @copyright Copyright (c) 2024
 */

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"

using namespace std;
using namespace Quadruped;

namespace {

const double kHipAngle = 0.3;
const double kThighAngle = 1.1;
const double kCalfAngle = -2.2;
const double kMaxTimeSeconds = 300.0;
const string kRobotName = "a1_gazebo";
const string kUrdfParamName = "robot_description";
const string kBaseLinkName = "a1_gazebo::base";
const int kLoopRate1Hz = 1000;
const int kLoopRate2Hz = 500;

/**
 * @brief 设置机器人模型状态
 * @param modelStateClient ROS服务客户端
 * @param modelState 机器人模型状态
 * @return 设置是否成功
 * @details 调用Gazebo服务设置机器人模型的状态
 */
bool SetModelState(ros::ServiceClient& modelStateClient, const gazebo_msgs::ModelState& modelState) {
  gazebo_msgs::SetModelState set_model_state;
  set_model_state.request.model_state = modelState;
  if (modelStateClient.call(set_model_state) && set_model_state.response.success) {
    return true;
  } else {
    ROS_ERROR("Failed to set model state.");
    return false;
  }
}

/**
 * @brief 设置机器人关节状态
 * @param jointStateClient ROS服务客户端
 * @param modelName 机器人模型名称
 * @param jointNames 关节名称列表
 * @param jointPositions 关节位置列表
 * @return 设置是否成功
 * @details 调用Gazebo服务设置机器人关节的状态
 */
bool SetJointState(
    ros::ServiceClient& jointStateClient,
    const string& modelName,
    const vector<string>& jointNames,
    const vector<double>& jointPositions) {
  gazebo_msgs::SetModelConfiguration set_joint_state;
  set_joint_state.request.model_name = modelName;
  set_joint_state.request.urdf_param_name = kUrdfParamName;
  set_joint_state.request.joint_names = jointNames;
  set_joint_state.request.joint_positions = jointPositions;
  if (jointStateClient.call(set_joint_state) && set_joint_state.response.success) {
    return true;
  } else {
    ROS_ERROR("Failed to set joints");
    return false;
  }
}

/**
 * @brief 重置机器人
 * @param modelStateClient ROS服务客户端
 * @param jointStateClient ROS服务客户端
 * @return 重置是否成功
 * @details 初始化机器人的模型和关节状态
 */
bool ResetRobot(ros::ServiceClient& modelStateClient, ros::ServiceClient& jointStateClient) {
  gazebo_msgs::ModelState model_state;
  model_state.model_name = kRobotName;
  model_state.reference_frame = "world";
  model_state.pose.position.x = 0;
  model_state.pose.position.y = 0;
  model_state.pose.position.z = 0.3;
  model_state.pose.orientation.w = 1;
  model_state.twist.linear.x = 0;
  model_state.twist.linear.y = 0;
  model_state.twist.linear.z = 0;
  model_state.twist.angular.x = 0;
  model_state.twist.angular.y = 0;
  model_state.twist.angular.z = 0;

  vector<string> joint_names = {"FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
                                "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
                                "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};

  vector<double> joint_positions = {-kHipAngle, kThighAngle, kCalfAngle, kHipAngle, kThighAngle, kCalfAngle,
                                    -kHipAngle, kThighAngle, kCalfAngle, kHipAngle, kThighAngle, kCalfAngle};

  return SetModelState(modelStateClient, model_state) &&
         SetJointState(jointStateClient, kRobotName, joint_names, joint_positions);
}

/**
 * @brief 获取机器人在世界坐标系中的质心位置
 * @param quadruped 四足机器人指针
 * @param baseStateClient ROS服务客户端
 * @details 调用Gazebo服务获取机器人质心位置和姿态，并更新机器人状态
 */
void GetComPositionInWorldFrame(qrRobot* quadruped, ros::ServiceClient& baseStateClient) {
  gazebo_msgs::GetLinkState get_link_state;
  get_link_state.request.link_name = kBaseLinkName;
  get_link_state.request.reference_frame = "world";

  if (baseStateClient.call(get_link_state) && get_link_state.response.success) {
    const auto& pose = get_link_state.response.link_state.pose;
    const auto& twist = get_link_state.response.link_state.twist;

    Vec3<double> pos_in = {pose.position.x, pose.position.y, pose.position.z};
    Quat<double> orientation_in = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    Vec3<double> v_in = {twist.linear.x, twist.linear.y, twist.linear.z};

    quadruped->gazeboBasePosition = pos_in.cast<float>();
    quadruped->gazeboBaseOrientation = orientation_in.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = v_in.cast<float>();
    quadruped->gazeboFootPositionInWorldFrame =
        quadruped->GetFootPositionsInWorldFrame(true, pos_in.cast<float>(), orientation_in.cast<float>());
  } else {
    ROS_ERROR("Failed to get link state.");
  }
}

/**
 * @brief 初始化机器人
 * @param nh ROS节点句柄
 * @param private_nh 私有ROS节点句柄
 * @return 初始化后的四足机器人指针
 * @details 创建并初始化四足机器人对象
 */
qrRobot* InitializeRobot(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  std::string home_dir = ros::package::getPath("quadruped") + "/";
  qrRobot* quadruped = new qrRobotA1Sim(nh, private_nh, home_dir + "config/a1_sim/a1_sim.yaml");
  quadruped->Step(Eigen::Matrix<float, 5, 12>::Zero(), HYBRID_MODE);
  quadruped->ReceiveObservation();

  Visualization2D& vis = quadruped->stateDataFlow.visualizer;
  vis.SetLabelNames({"pitch", "H", "vx in world", "vy in world", "vz in world"});

  ROS_INFO("Robot initialized.");
  return quadruped;
}

/**
 * @brief 主控制循环
 * @param quadruped 四足机器人指针
 * @param robot_runner 机器人运行器
 * @param baseStateClient ROS服务客户端
 * @details 控制四足机器人进行运动，并监控其状态
 */
void ControlLoop(qrRobot* quadruped, qrRobotRunner& robot_runner, ros::ServiceClient& baseStateClient) {
  float start_time = quadruped->GetTimeSinceReset();
  float current_time = start_time;
  float start_time_wall = start_time;

  Eigen::Matrix<float, 12, 1> motor_angles_after_keep_stand = quadruped->GetMotorAngles();
  for (int leg_id = 0; leg_id < 4; ++leg_id) {
    motor_angles_after_keep_stand[3 * leg_id + 0] = 0.;
    motor_angles_after_keep_stand[3 * leg_id + 1] = 1.2;
    motor_angles_after_keep_stand[3 * leg_id + 2] = -2.4;
  }

  ROS_INFO("Starting control loop...");
  int count = 0;
  float avg_cost = 0;

  ros::Rate loop_rate1(kLoopRate1Hz);
  ros::Rate loop_rate2(kLoopRate2Hz);

  while (ros::ok() && current_time - start_time < kMaxTimeSeconds) {
    start_time_wall = quadruped->GetTimeSinceReset();
    robot_runner.Update();
    robot_runner.Step();

    current_time = quadruped->GetTimeSinceReset();
    avg_cost += (current_time - start_time_wall);
    if ((count + 1) % 1000 == 0) {
      ROS_INFO("Average time cost = %f", avg_cost / 1000);
      avg_cost = 0;
    }
    ++count;
    loop_rate1.sleep();
    loop_rate2.sleep();
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "a1_sim");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::ServiceClient model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::ServiceClient joint_state_client =
      nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  ros::ServiceClient base_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  if (!ResetRobot(model_state_client, joint_state_client)) {
    ROS_ERROR("Failed to reset robot.");
    return -1;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("ROS node initialized.");

  qrRobot* quadruped = InitializeRobot(nh, private_nh);
  qrRobotRunner robot_runner(quadruped, nh);

  GetComPositionInWorldFrame(quadruped, base_state_client);

  ControlLoop(quadruped, robot_runner, base_state_client);

  return 0;
}
