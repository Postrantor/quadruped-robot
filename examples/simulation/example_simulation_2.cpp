/**
 * @author GPT4-o
 * @brief
 * @date 2024-06-24 16:12:43
 * @copyright Copyright (c) 2024
 */

#include <future>
#include <Eigen/Dense>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_srvs/srv/empty.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/set_model_state.hpp"
#include "gazebo_msgs/srv/set_model_configuration.hpp"

#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/utils/qr_cpptypes.h"
// #include "quadruped/ros/qr_control2gazebo.h"

/**
 * @brief 重置机器人的姿态和关节状态
 * @details 通过调用Gazebo服务重置机器人的位置、姿态和关节角度
 * @param model_state_client 设置模型状态的服务客户端
 * @param joint_state_client 设置关节状态的服务客户端
 * @return 是否成功重置机器人的状态
 * @note 通过 `ros2 service type <service_name>` 查看对应服务类型
 *  按照相同的类型对比 example 实例程序中的 srv
 *  需要显示加载 `gazebo_ros_state` 插件
 */
auto reset_robot(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetEntityState>>& entity_state_client,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>>& joint_state_client) -> bool {
  // set request
  auto entity_state_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  auto joint_state_request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();

  // 设置实体状态请求参数
  entity_state_request->state.name = "a1_gazebo::base";
  entity_state_request->state.reference_frame = "world";
  entity_state_request->state.pose.position.x = 0;
  entity_state_request->state.pose.position.y = 0;
  entity_state_request->state.pose.position.z = 0.3;
  entity_state_request->state.pose.orientation.w = 1;

  // 设置关节状态请求参数
  joint_state_request->model_name = "a1_gazebo";
  joint_state_request->urdf_param_name = "robot_description";
  joint_state_request->joint_names = {
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
  // 定义关节角度
  double hip_angle = 0.3;
  double thigh_angle = 1.1;
  double calf_angle = -2.2;
  joint_state_request->joint_positions = {
      -hip_angle,   //
      thigh_angle,  //
      calf_angle,   //
      hip_angle,    //
      thigh_angle,  //
      calf_angle,   //
      -hip_angle,   //
      thigh_angle,  //
      calf_angle,   //
      hip_angle,    //
      thigh_angle,  //
      calf_angle};

  // 异步发送设置实体状态和关节状态请求
  auto entity_state_response_future = entity_state_client->async_send_request(entity_state_request);
  auto joint_state_response_future = joint_state_client->async_send_request(joint_state_request);

  // 等待并检查响应结果
  if (rclcpp::spin_until_future_complete(
          node, entity_state_response_future, std::chrono::duration<double, std::milli>(5000)) ==
          rclcpp::FutureReturnCode::SUCCESS  //
      && rclcpp::spin_until_future_complete(
             node, joint_state_response_future, std::chrono::duration<double, std::milli>(5000)) ==
             rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(node->get_logger(), "robot reset successfully");
    return true;
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "failed to reset the robot");
    return false;
  }

  return false;
}

/**
 * @brief 获取机器人在世界坐标系中的位姿
 * @details 通过调用Gazebo服务获取机器人的位姿和速度
 * @param quadruped 指向机器人实例的指针
 * @param base_state_client 获取链接状态的服务客户端
 */
void get_com_position_in_world_frame(
    Quadruped::qrRobot* quadruped,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetEntityState>>& base_state_client) {
  // send request
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "a1_gazebo::base";
  request->reference_frame = "world";

  // 异步发送获取链接状态请求
  auto response_future = base_state_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, response_future, std::chrono::duration<double, std::milli>(5000))  //
      == rclcpp::FutureReturnCode::SUCCESS) {
    const auto& response = response_future.get();
    if (!response->success) {
      RCLCPP_WARN_STREAM(node->get_logger(), "failed to get gazebo link state");
      return;
    }

    // 提取位姿和速度信息
    const auto& pose = response->state.pose;
    const auto& twist = response->state.twist;

    Vec3<double> pos_in{pose.position.x, pose.position.y, pose.position.z};
    Vec3<double> v_in{twist.linear.x, twist.linear.y, twist.linear.z};
    Quat<double> orientation_in{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};

    // 更新机器人实例的状态信息
    quadruped->gazeboBasePosition = pos_in.cast<float>();
    quadruped->gazeboBaseOrientation = orientation_in.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = v_in.cast<float>();
    // 获取并更新机器人足端的位置
    quadruped->gazeboFootPositionInWorldFrame =  //
        quadruped->GetFootPositionsInWorldFrame(
            true,                  //
            pos_in.cast<float>(),  //
            orientation_in.cast<float>());
  } else {
    RCLCPP_ERROR(node->get_logger(), "call `GetEntityState` service failed");
  }
}

/**
 * @brief 初始化并重置机器人
 * @details 初始化ROS节点和服务客户端，重置机器人姿态
 * @param node 指向ROS节点的指针
 * @return 指向Quadruped机器人实例的智能指针
 */
std::unique_ptr<Quadruped::qrRobotA1Sim> initialize_and_reset_robot(const std::shared_ptr<rclcpp::Node>& node) {
  RCLCPP_INFO_STREAM(node->get_logger(), "initialize and reset robot.");
  // 创建服务客户端
  // 服务`/gazebo/set_entity_state`由"gazebo_ros_state"插件配置
  // 在empty.world文件中通过remap参数进行重映射或者添加命名空间得到
  auto entity_state_client = node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  // 这个服务在ros2版本上还是没有？
  auto joint_state_client =
      node->create_client<gazebo_msgs::srv::SetModelConfiguration>("/gazebo/set_model_configuration");

  // 等待服务可用
  if (!entity_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "model state service not available");
    // rclcpp::shutdown();
    // throw std::runtime_error("model state service not available");
  }
  if (!joint_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "joint state service not available");
    // rclcpp::shutdown();
    // throw std::runtime_error("joint state service not available");
  }

  // 重置机器人状态
  if (!reset_robot(node, entity_state_client, joint_state_client)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "reset robot failed.");
    // rclcpp::shutdown();
    // throw std::runtime_error("failed to reset the robot");
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "reset the robot pose success.");

  // 获取功能包 "quadruped" 的共享目录路径
  std::string package_path;
  try {
    package_path = ament_index_cpp::get_package_share_directory("quadruped");
    RCLCPP_INFO_STREAM(node->get_logger(), "find package path in: \n\t- `" << package_path << "`");
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    return nullptr;
  }
  // 构建配置文件路径
  std::string config_file_path = package_path + "/../../config/a1_sim/a1_sim.yaml";
  RCLCPP_INFO_STREAM(node->get_logger(), "config file path: \n\t- `" << config_file_path << "`");

  // 创建并返回机器人实例
  return std::make_unique<Quadruped::qrRobotA1Sim>(node, config_file_path);
}

/**
 * @brief 主控制循环
 * @details 执行机器人控制逻辑
 * @param quadruped 指向Quadruped机器人实例的指针
 * @param node 指向ROS节点的指针
 */
void control_loop(
    Quadruped::qrRobotA1Sim* quadruped,  //
    const std::shared_ptr<rclcpp::Node>& node) {
  RCLCPP_INFO_STREAM(node->get_logger(), "entry control loop.");
  // 1. 初始化机器人并接收初始观测数据
  quadruped->Step(Eigen::Matrix<float, 5, 12>::Zero(), Quadruped::HYBRID_MODE);
  quadruped->ReceiveObservation();
  // 机器人的基础朝向信息
  RCLCPP_INFO_STREAM(
      node->get_logger(),
      "\n\t"
          << "base orientation: [" << quadruped->GetBaseOrientation().x()  //
          << ", " << quadruped->GetBaseOrientation().y()                   //
          << ", " << quadruped->GetBaseOrientation().z()                   //
          << ", " << quadruped->GetBaseOrientation().w() << "]");
  // 设置可视化标签
  // Quadruped::Visualization2D& vis = quadruped->stateDataFlow.visualizer;
  // vis.SetLabelNames({"pitch", "H", "vx in world", "vy in world", "vz in world"});

  // 2. 创建服务客户端，获取初始状态
  auto base_state_client = node->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
  if (!base_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "base state service not available");
    // rclcpp::shutdown();
    // return;
  }
  get_com_position_in_world_frame(quadruped, node, base_state_client);

  // 3. 创建机器人运行实例
  std::string home_dir = ament_index_cpp::get_package_share_directory("quadruped") + "/../../";
  RCLCPP_INFO_STREAM(node->get_logger(), "get package config dir: \n\t- `" << home_dir << "`");
  qrRobotRunner robotRunner(quadruped, home_dir, node);

  // 4. 创建并初始化控制器消息传递实例
  // auto controller2gazeboMsg = std::make_unique<Quadruped::qrController2GazeboMsg>(
  //     quadruped,                              //
  //     robotRunner.GetLocomotionController(),  //
  //     node);
  // RCLCPP_INFO(node->get_logger(), "ros2 modules init finished");

  // 5. main loop
  rclcpp::Rate loop_rate1(1000);  // 设置循环频率
  rclcpp::Rate loop_rate2(700);

  float startTime = quadruped->GetTimeSinceReset();
  float currentTime = startTime;
  float startTimeWall = startTime;

  while (rclcpp::ok() && currentTime - startTime < 1000.0f) {
    startTimeWall = quadruped->GetTimeSinceReset();

    robotRunner.Update();
    robotRunner.Step();

    currentTime = quadruped->GetTimeSinceReset();

    // 机器人状态检查
    if (quadruped->basePosition[2] < 0.10 ||                     //
        quadruped->stateDataFlow.heightInControlFrame < 0.05 ||  //
        quadruped->basePosition[2] > 0.40 ||                     //
        std::abs(quadruped->baseRollPitchYaw[0]) > 0.6) {
      RCLCPP_INFO_STREAM(
          node->get_logger(), "\n\t"
                                  << "base pos:" << quadruped->basePosition << "\n\t"
                                  << "base rpy:" << quadruped->GetBaseRollPitchYaw());
      RCLCPP_ERROR(node->get_logger(), "the dog is going down, main function exit.");
      break;
    }

    // 根据时间控制节奏
    // FIXME(@zhiqi.jia) :: use timer callback
    if (quadruped->useRosTime) {
      rclcpp::spin_some(node);  // 非阻塞回调执行
      if (quadruped->timeStep < 0.0015)
        loop_rate1.sleep();
      else
        loop_rate2.sleep();
    } else {
      while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {
      }
    }
  }

  // quadruped->stateDataFlow.visualizer.Show();
}

/**
 * @brief 主函数
 * @details 初始化ROS节点，执行主控制循环
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("a1_sim");
  RCLCPP_INFO_STREAM(node->get_logger(), "start robot_sim node.");

  try {
    control_loop(initialize_and_reset_robot(node).get(), node);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
