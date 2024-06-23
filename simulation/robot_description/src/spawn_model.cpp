/**
 * @author GPT4-o
 * @date 2024-06-23 11:17:42
 * @brief 主函数，使用 ROS2 在 Gazebo 中生成和管理机器人模型
 * @details 本程序初始化 ROS2，创建 GazeboSpawner 对象，并管理机器人模型的生命周期，包括生成、控制和删除。
 * @date 2024-06-23
 */

/**
 * `roslaunch qr_gazebo gazebo_startup.launch wname:=earth`
 * `roslaunch qr_gazebo model_spawn.launch rname:=a1 use_xacro:=true use_camera:=false`
 */

#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "qr_gazebo/gazebo_model_spawn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

/**
 * @brief 等待用户输入
 * @param message 等待输入前显示的消息
 */
void wait_for_user_input(const std::string& message) {
  std::cout << message << std::endl;
  std::cin.get();
}

/**
 * @brief 设置并生成机器人模型
 * @param manager GazeboSpawner 管理器
 * @param robot_type 机器人类型
 * @return 是否成功生成模型
 */
bool setup_and_spawn_model(GazeboSpawner& manager, const std::string& robot_type) {
  manager.set_model_position(0, 0, 0.4);
  manager.set_model_orientation(0, 0, 0, 0);

  if (!manager.spawn_model("robot_description")) {
    RCLCPP_ERROR(manager.getNode()->get_logger(), "fail to spawn model in gazebo: %s", robot_type.c_str());
    return false;
  }
  return true;
}

/**
 * @brief 主函数，运行 ROS2 节点并管理 Gazebo 模型
 * @param argc 命令行参数的数量
 * @param argv 命令行参数的数组
 * @return 退出状态
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spawn_model");

  if (argc < 2) {
    RCLCPP_ERROR(node->get_logger(), "please specify the robot type.");
    std::exit(1);
  }
  const std::string robot_type = argv[1];

  GazeboSpawner manager(robot_type, node);
  RCLCPP_INFO(node->get_logger(), "Robot Type: %s", manager.getRobotType().c_str());
  RCLCPP_INFO(node->get_logger(), "Node Name: %s", manager.getNode()->get_name());

  if (!setup_and_spawn_model(manager, robot_type)) {
    RCLCPP_ERROR(node->get_logger(), "fail to spawn model in gazebo: %s", robot_type.c_str());
    return 1;
  }

  wait_for_user_input("press enter key to start controllers.");
  manager.load_controllers();
  manager.start_controllers();

  wait_for_user_input("press enter key to delete controllers and model.");
  if (!manager.stop_controllers()) {
    RCLCPP_ERROR(node->get_logger(), "failed to stop controllers in gazebo: %s", robot_type.c_str());
    return 1;
  }
  manager.unload_controllers();
  manager.delete_model();

  rclcpp::shutdown();
  return 0;
}
