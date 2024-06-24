/**
 * @author GPT4-o
 * @brief
 * @date 2024-05-29
 * @copyright Copyright (c) 2024
 */

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/set_model_configuration.hpp"
#include "std_srvs/srv/empty.hpp"

#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "utils/qr_cpptypes.h"

namespace {

const double kHipAngle = 0.3;
const double kThighAngle = 1.1;
const double kCalfAngle = -2.2;
const double kMaxTimeSeconds = 300.0;
const std::string kRobotName = "a1_gazebo";
const std::string kUrdfParamName = "robot_description";
const std::string kBaseLinkName = "a1_gazebo::base";
const int kLoopRate1Hz = 1000;
const int kLoopRate2Hz = 500;

/**
 * @brief 设置机器人模型状态
 * @param client ROS服务客户端
 * @param model_state 机器人模型状态
 * @return 设置是否成功
 * @details 调用Gazebo服务设置机器人模型的状态
 */
bool SetModelState(
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client,
    const gazebo_msgs::msg::EntityState& model_state) {
  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state = model_state;

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("set_model_state_node"), result)  //
      == rclcpp::FutureReturnCode::SUCCESS) {
    return result.get()->success;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("set_model_state_node"), "Failed to set model state");
    return false;
  }
}

/**
 * @brief 设置机器人关节状态
 * @param client ROS服务客户端
 * @param joint_names 关节名称列表
 * @param joint_positions 关节位置列表
 * @return 设置是否成功
 * @details 调用Gazebo服务设置机器人各关节的状态
 */
bool SetJointState(
    rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedPtr client,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_positions) {
  auto request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();
  request->model_name = kRobotName;
  request->urdf_param_name = kUrdfParamName;
  request->joint_names = joint_names;
  request->joint_positions = joint_positions;

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("set_joint_state_node"), result)  //
      == rclcpp::FutureReturnCode::SUCCESS) {
    return result.get()->success;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("set_joint_state_node"), "Failed to set joint state");
    return false;
  }
}

/**
 * @brief 重置机器人状态
 * @param model_state_client Gazebo设置模型状态服务客户端
 * @param joint_state_client Gazebo设置关节状态服务客户端
 * @return 重置是否成功
 */
bool ResetRobot(
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr model_state_client,
    rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedPtr joint_state_client) {
  gazebo_msgs::msg::EntityState model_state;
  model_state.name = kRobotName;
  model_state.reference_frame = "world";

  geometry_msgs::msg::Twist model_twist;
  geometry_msgs::msg::Pose model_pose;

  model_twist.linear.x = 0.0;
  model_twist.linear.y = 0.0;
  model_twist.linear.z = 0.0;
  model_twist.angular.x = 0.0;
  model_twist.angular.y = 0.0;
  model_twist.angular.z = 0.0;

  model_pose.position.x = 0;
  model_pose.position.y = 0;
  model_pose.position.z = 0.3;
  model_pose.orientation.w = 1;
  model_pose.orientation.x = 0;
  model_pose.orientation.y = 0;
  model_pose.orientation.z = 0;

  model_state.twist = model_twist;
  model_state.pose = model_pose;

  if (!SetModelState(model_state_client, model_state)) {
    return false;
  }

  std::vector<std::string> joint_names = {
      "FR_hip_joint",    //
      "FR_thigh_joint",  //
      "FR_calf_joint",   //
      "FL_hip_joint",    //
      "FL_thigh_joint",  //
      "FL_calf_joint",   //
      "RR_hip_joint",    //
      "RR_thigh_joint",  //
      "RR_calf_joint",   //
      "RL_hip_joint",    //
      "RL_thigh_joint",  //
      "RL_calf_joint"};

  std::vector<double> joint_positions = {
      -kHipAngle,   //
      kThighAngle,  //
      kCalfAngle,   //
      kHipAngle,    //
      kThighAngle,  //
      kCalfAngle,
      -kHipAngle,   //
      kThighAngle,  //
      kCalfAngle,   //
      kHipAngle,    //
      kThighAngle,  //
      kCalfAngle};

  return SetJointState(joint_state_client, joint_names, joint_positions);
}

/**
 * @brief 获取机器人在世界坐标系中的位置
 * @param quadruped 机器人实例
 * @param base_state_client Gazebo获取链接状态服务客户端
 */
void GetComPositionInWorldFrame(
    qrRobot* quadruped,  //
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr base_state_client) {
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = kBaseLinkName;
  request->reference_frame = "world";

  auto result = base_state_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("get_com_position_node"), result)  //
      == rclcpp::FutureReturnCode::SUCCESS) {
    const auto& pose = result.get()->state.pose;
    const auto& twist = result.get()->state.twist;
    Vec3<double> pos_in = {pose.position.x, pose.position.y, pose.position.z};
    Quat<double> orientation_in = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    Vec3<double> v_in = {twist.linear.x, twist.linear.y, twist.linear.z};

    quadruped->gazeboBasePosition = pos_in.cast<float>();
    quadruped->gazeboBaseOrientation = orientation_in.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = v_in.cast<float>();

    quadruped->gazeboFootPositionInWorldFrame =
        quadruped->GetFootPositionsInWorldFrame(true, pos_in.cast<float>(), orientation_in.cast<float>());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("get_com_position_node"), "Failed to get COM position in world frame");
  }
}

/**
 * @brief 初始化机器人
 * @param nh ROS2节点句柄
 * @param private_nh ROS2私有节点句柄
 * @return 初始化的机器人实例
 */
qrRobot* InitializeRobot(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh) {
  std::string home_dir = std::getenv("HOME");
  std::string config_path = home_dir + "/quadruped/config/a1_sim/a1_sim.yaml";
  return new qrRobotA1Sim(nh, private_nh, config_path);
}

/**
 * @brief 控制主循环
 * @param quadruped 机器人实例
 * @param robot_runner 机器人运行实例
 * @param base_state_client Gazebo获取链接状态服务客户端
 */
void ControlLoop(
    qrRobot* quadruped,
    qrRobotRunner& robot_runner,
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr base_state_client) {
  rclcpp::Rate loop_rate1(kLoopRate1Hz);
  rclcpp::Rate loop_rate2(kLoopRate2Hz);

  float start_time = quadruped->GetTimeSinceReset();
  float current_time = start_time;
  float avg_cost = 0;
  int count = 0;

  while (rclcpp::ok() && current_time - start_time < kMaxTimeSeconds) {
    float start_time_wall = quadruped->GetTimeSinceReset();

    robot_runner.Update();
    robot_runner.Step();

    current_time = quadruped->GetTimeSinceReset();
    avg_cost += (current_time - start_time_wall);

    if ((count + 1) % 1000 == 0) {
      RCLCPP_INFO(rclcpp::get_logger("control_loop"), "Avg time cost = %f [ms]", avg_cost / 1000);
      avg_cost = 0;
    }

    if (quadruped->useRosTime) {
      rclcpp::spin_some(rclcpp::Node::make_shared("control_loop"));
      loop_rate1.sleep();
      loop_rate2.sleep();
    } else {
      while (quadruped->GetTimeSinceReset() - start_time_wall < quadruped->timeStep) {
      }
    }

    ++count;
  }
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("a1_sim");
  auto private_nh = rclcpp::Node::make_shared("~");

  auto model_state_client = nh->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  auto joint_state_client =
      nh->create_client<gazebo_msgs::srv::SetModelConfiguration>("/gazebo/set_model_configuration");
  auto base_state_client = nh->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");

  if (!ResetRobot(model_state_client, joint_state_client)) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to reset robot.");
    return -1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.add_node(private_nh);
  executor.spin();

  RCLCPP_INFO(nh->get_logger(), "ROS node initialized.");

  qrRobot* quadruped = InitializeRobot(nh, private_nh);
  qrRobotRunner robot_runner(quadruped, nh);

  GetComPositionInWorldFrame(quadruped, base_state_client);

  ControlLoop(quadruped, robot_runner, base_state_client);

  rclcpp::shutdown();
  return 0;
}
