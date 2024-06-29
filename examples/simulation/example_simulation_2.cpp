/**
 * @author GPT4-o
 * @brief
 * @date 2024-06-24 16:12:43
 * @copyright Copyright (c) 2024
 */

#include <future>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_srvs/srv/empty.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/set_model_configuration.hpp"

#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "utils/qr_cpptypes.h"

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
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetModelState>>& model_state_client,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>>& joint_state_client) -> bool {
  auto model_state_request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
  auto joint_state_request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();

  // 设置模型状态请求参数
  model_state_request->model_state.model_name = "a1_gazebo";
  model_state_request->model_state.reference_frame = "world";
  model_state_request->model_state.twist = {};
  model_state_request->model_state.pose.position.x = 0;
  model_state_request->model_state.pose.position.y = 0;
  model_state_request->model_state.pose.position.z = 0.3;
  model_state_request->model_state.pose.orientation.w = 1;

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

  // 异步发送设置模型状态和关节状态请求
  auto model_state_response_future = model_state_client->async_send_request(model_state_request);
  auto joint_state_response_future = joint_state_client->async_send_request(joint_state_request);

  // 等待并检查响应结果
  if (rclcpp::spin_until_future_complete(
          model_state_client->get_node_base_interface(),  //
          model_state_response_future) == rclcpp::FutureReturnCode::SUCCESS &&
      rclcpp::spin_until_future_complete(
          joint_state_client->get_node_base_interface(),  //
          joint_state_response_future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("unitree_robot"), "robot reset successfully");
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "failed to reset the robot");
    return false;
  }
}

/**
 * @brief 获取机器人在世界坐标系中的位姿
 * @details 通过调用Gazebo服务获取机器人的位姿和速度
 * @param quadruped 指向机器人实例的指针
 * @param base_state_client 获取链接状态的服务客户端
 */
void get_com_position_in_world_frame(
    Quadruped::qrRobot* quadruped,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetEntityState>>& base_state_client) {
  // send request
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->link_name = "a1_gazebo::base";
  request->reference_frame = "world";

  // 异步发送获取链接状态请求
  auto response_future = base_state_client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(base_state_client->get_node_base_interface(), response_future)  //
      == rclcpp::FutureReturnCode::SUCCESS) {
    const auto& response = response_future.get();
    if (!response->success) {
      RCLCPP_WARN(rclcpp::get_logger("unitree_robot"), "Failed to get Gazebo link state");
      return;
    }

    // 提取位姿和速度信息
    const auto& pose = response->link_state.pose;
    const auto& twist = response->link_state.twist;

    Vec3<double> pos_in{pose.position.x, pose.position.y, pose.position.z};
    Quat<double> orientation_in{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    Vec3<double> v_in{twist.linear.x, twist.linear.y, twist.linear.z};

    // 更新机器人实例的状态信息
    quadruped->gazeboBasePosition = pos_in.cast<float>();
    quadruped->gazeboBaseOrientation = orientation_in.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = v_in.cast<float>();

    // 获取并更新机器人足端的位置
    quadruped->gazeboFootPositionInWorldFrame =
        quadruped->GetFootPositionsInWorldFrame(true, pos_in.cast<float>(), orientation_in.cast<float>());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "service call failed");
  }
}

/**
 * @brief 初始化并重置机器人
 * @details 初始化ROS节点和服务客户端，重置机器人姿态
 * @param node 指向ROS节点的指针
 * @return 指向Quadruped机器人实例的智能指针
 */
std::unique_ptr<Quadruped::qrRobotA1Sim> initialize_and_reset_robot(const std::shared_ptr<rclcpp::Node>& node) {
  // 创建服务客户端
  // 这里指定的服务名称`/gazebo/set_entity_state`是应该通过"gazebo_ros_state"插件配置的
  // 在empty.world文件中通过remap参数进行重映射或者添加命名空间得到
  auto model_state_client = node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  // 这个服务在ros2版本上还是没有？
  auto joint_state_client =
      node->create_client<gazebo_msgs::srv::SetModelConfiguration>("/gazebo/set_model_configuration");

  // 等待服务可用
  if (!model_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "model state service not available");
    rclcpp::shutdown();
    throw std::runtime_error("model state service not available");
  }

  if (!joint_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "joint state service not available");
    rclcpp::shutdown();
    throw std::runtime_error("joint state service not available");
  }

  // 重置机器人状态
  if (!reset_robot(model_state_client, joint_state_client)) {
    rclcpp::shutdown();
    throw std::runtime_error("failed to reset the robot");
  }

  RCLCPP_INFO(rclcpp::get_logger("unitree_robot"), "reset the robot pose");

  // 获取功能包 "quadruped" 的共享目录路径
  std::string package_path;
  try {
    package_path = ament_index_cpp::get_package_share_directory("quadruped");
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  // 构建配置文件路径
  std::string config_file_path = package_path + "/config/a1_sim/a1_sim.yaml";
  std::cout << "Config file path: " << config_file_path << std::endl;

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
  // 1. 初始化机器人并接收初始观测数据
  quadruped->Step(Eigen::Matrix<float, 5, 12>::Zero(), Quadruped::HYBRID_MODE);
  quadruped->ReceiveObservation();
  // 输出机器人的基础朝向信息
  RCLCPP_INFO(
      rclcpp::get_logger("unitree_robot"), "Base Orientation: %f %f %f %f",  //
      quadruped->GetBaseOrientation().x(),                                   //
      quadruped->GetBaseOrientation().y(),                                   //
      quadruped->GetBaseOrientation().z(),                                   //
      quadruped->GetBaseOrientation().w());
  // 设置可视化标签
  Quadruped::Visualization2D& vis = quadruped->stateDataFlow.visualizer;
  vis.SetLabelNames({"pitch", "H", "vx in world", "vy in world", "vz in world"});

  // 2. 创建机器人运行实例
  std::string home_dir = ament_index_cpp::get_package_share_directory("quadruped");
  Quadruped::qrRobotRunner robotRunner(quadruped, home_dir, node);

  // 3. 创建并初始化控制器消息传递实例
  auto controller2gazeboMsg = std::make_unique<Quadruped::qrController2GazeboMsg>(
      quadruped,                              //
      robotRunner.GetLocomotionController(),  //
      node);
  RCLCPP_INFO(rclcpp::get_logger("unitree_robot"), "ros2 modules init finished");

  // 3. 创建服务客户端，获取初始状态
  auto base_state_client = node->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
  if (!base_state_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "base state service not available");
    rclcpp::shutdown();
    return;
  }
  get_com_position_in_world_frame(quadruped, base_state_client);

  // 4. 主循环
  rclcpp::Rate loop_rate1(1000);  // 设置循环频率
  rclcpp::Rate loop_rate2(700);

  float startTime = quadruped->GetTimeSinceReset();
  float currentTime = startTime;
  float startTimeWall = startTime;

  while (rclcpp::ok() && currentTime - startTime < Quadruped::MAX_TIME_SECONDS) {
    startTimeWall = quadruped->GetTimeSinceReset();

    robotRunner.Update();
    robotRunner.Step();

    currentTime = quadruped->GetTimeSinceReset();

    // 机器人状态检查
    if (quadruped->basePosition[2] < 0.10 ||                     //
        quadruped->stateDataFlow.heightInControlFrame < 0.05 ||  //
        quadruped->basePosition[2] > 0.40 ||                     //
        std::abs(quadruped->baseRollPitchYaw[0]) > 0.6) {
      RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "the dog is going down, main function exit.");
      std::cout << "base pos:" << quadruped->basePosition << std::endl;
      std::cout << "base rpy:" << quadruped->GetBaseRollPitchYaw() << std::endl;
      break;
    }

    // 根据时间控制节奏
    // FIXME(@zhiqi.jia) :: use timer
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

  quadruped->stateDataFlow.visualizer.Show();
}

/**
 * @brief 主函数
 * @details 初始化ROS节点，执行主控制循环
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("a1_sim");

  try {
    auto quadruped = initialize_and_reset_robot(node);
    control_loop(quadruped.get(), node);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_robot"), "%s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
