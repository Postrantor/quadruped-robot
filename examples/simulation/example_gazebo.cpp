/**
 * @brief
 * @author postrantor@gmail.com
 * @date 2024-08-03 20:22:37
 * @copyright Copyright (c) 2024
 */

#include <future>
#include <chrono>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_srvs/srv/empty.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/utils/qr_cpptypes.h"
// #include "quadruped/ros/qr_control2gazebo.h"

/**
 * @brief 重置机器人的姿态和关节状态
 * @note 需要显示加载 `gazebo_ros_state` 插件
 * @param model_state_client 设置模型状态的服务客户端
 * @param joint_state_client 设置关节状态的服务客户端
 * @return 是否成功重置机器人的状态
 */
bool reset_robot(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetEntityState>>& entity_state_client) {
  // set request
  auto entity_state_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

  entity_state_request->state.name = "robot_a1";
  entity_state_request->state.reference_frame = "ground_plane";

  entity_state_request->state.pose.position.x = 0.0;
  entity_state_request->state.pose.position.y = 0.0;
  entity_state_request->state.pose.position.z = 0.0;

  // It should match the initial value in quadruped.
  entity_state_request->state.pose.orientation.w = 1.0;
  entity_state_request->state.pose.orientation.x = 0.0;
  entity_state_request->state.pose.orientation.y = 0.0;
  entity_state_request->state.pose.orientation.z = 0.0;

  entity_state_request->state.twist.linear.x = 0.0;
  entity_state_request->state.twist.linear.y = 0.0;
  entity_state_request->state.twist.linear.z = 0.0;

  entity_state_request->state.twist.angular.x = 0.0;
  entity_state_request->state.twist.angular.y = 0.0;
  entity_state_request->state.twist.angular.z = 0.0;

  // asynchronously send requests to set entity state and joint state
  auto result_future = entity_state_client->async_send_request(entity_state_request);

  // check request result
  if (rclcpp::spin_until_future_complete(
          node,           //
          result_future,  //
          std::chrono::duration<double, std::milli>(2000)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO_STREAM(node->get_logger(), "joint state set successfully");
      return true;
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "failed to set joint state");
    }
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "failed to call service /set_entity_state");
  }

  return false;
}

/**
 * @brief 获取机器人在世界坐标系中的位姿
 * @param quadruped 指向机器人实例的指针
 * @param base_state_client 获取链接状态的服务客户端
 */
bool get_com_position_in_world_frame(
    Quadruped::qrRobot* quadruped,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetEntityState>>& base_state_client) {
  // send request
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "robot_a1";
  request->reference_frame = "ground_plane";

  auto response_future = base_state_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
          node,             //
          response_future,  //
          std::chrono::duration<double, std::milli>(2000)) == rclcpp::FutureReturnCode::SUCCESS) {
    const auto& response = response_future.get();
    if (!response->success) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "failed to get gazebo link state");
      return false;
    }

    // paser gazebo position and velocity information sent to quadruped object
    const auto& pose = response->state.pose;
    const auto& twist = response->state.twist;
    Vec3<double> pos_in{pose.position.x, pose.position.y, pose.position.z};
    Quat<double> orientation_in{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    Vec3<double> twist_in{twist.linear.x, twist.linear.y, twist.linear.z};

    // 更新机器人实例的状态信息
    quadruped->gazeboBasePosition = pos_in.cast<float>();
    quadruped->gazeboBaseOrientation = orientation_in.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = twist_in.cast<float>();
    // 获取并更新机器人足端的位置
    quadruped->gazeboFootPositionInWorldFrame =  //
        quadruped->GetFootPositionsInWorldFrame(
            true,                  //
            pos_in.cast<float>(),  //
            orientation_in.cast<float>());
  } else {
    RCLCPP_ERROR(node->get_logger(), "call /get_entity_state service failed");
    return false;
  }

  return true;
}

/**
 * @brief 初始化并重置机器人
 * @details 初始化ROS节点和服务客户端，重置机器人姿态
 * @param node 指向ROS节点的指针
 * @return 指向Quadruped机器人实例的智能指针
 */
std::unique_ptr<Quadruped::qrRobotA1Sim> initialize_and_reset_robot(const std::shared_ptr<rclcpp::Node>& node) {
  RCLCPP_INFO_STREAM(node->get_logger(), "initialize and reset robot.");
  auto entity_state_client = node->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

  // wait for service available
  if (!entity_state_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "/set_entity_state service not available");
  }

  // 重置机器人状态
  if (!reset_robot(node, entity_state_client)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "reset robot failed.");
  }

  // 构建配置文件路径
  std::string package_path_;
  try {
    package_path_ = ament_index_cpp::get_package_share_directory("quadruped");
    RCLCPP_INFO_STREAM(node->get_logger(), "find package path in: \n\t- `" << package_path_ << "`");
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    return nullptr;
  }
  std::string config_file_path_ = package_path_ + "/../../config/a1_sim/a1_sim.yaml";
  RCLCPP_INFO_STREAM(node->get_logger(), "config file path: \n\t- `" << config_file_path_ << "`");

  // 创建并返回机器人实例
  return std::make_unique<Quadruped::qrRobotA1Sim>(node, config_file_path_);
}

/**
 * @brief main control loop
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
  auto base_state_client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
  if (!base_state_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "base state service not available");
  }
  if (!get_com_position_in_world_frame(quadruped, node, base_state_client)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "get common position in world failed.");
  }

  // 3. 创建机器人运行实例
  std::string home_dir_ = ament_index_cpp::get_package_share_directory("quadruped") + "/../../";
  RCLCPP_INFO_STREAM(node->get_logger(), "get package config dir: \n\t- `" << home_dir_ << "`");
  qrRobotRunner robotRunner(quadruped, home_dir_, node);

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
                                  << "base pos: \n\t" << quadruped->basePosition << "\n\t"
                                  << "base rpy: \n\t" << quadruped->GetBaseRollPitchYaw());
      RCLCPP_WARN_STREAM(node->get_logger(), "the dog is going down, main function exit...");
      // break;
    }

    // 控制仿真频率
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

  const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_sim");
  RCLCPP_INFO_STREAM(node->get_logger(), "starting robot_sim node.");

  try {
    std::unique_ptr<Quadruped::qrRobotA1Sim> robot_sim = initialize_and_reset_robot(node);
    control_loop(robot_sim.get(), node);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
