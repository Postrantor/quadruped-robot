/**
 * @author GPT4-o
 * @brief
 * @date 2024-06-23 02:58:58
 * @copyright Copyright (c) 2024
 */

#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include "qr_gazebo/gazebo_model_spawn.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spawn_model");

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("spawn_model"), "Please specify the robot type.");
    return 1;
  }

  const char* type_cstr = argv[1];
  std::string robot_type(type_cstr);

  GazeboSpawner manager(robot_type, node);

  manager.set_model_position(0, 0, 0.4);
  manager.set_model_orientation(0, 0, 0, 0);

  if (!manager.spawn_model("robot_description")) {
    RCLCPP_ERROR(rclcpp::get_logger("spawn_model"), "Fail to spawn model in gazebo: %s", type_cstr);
    return 1;
  }

  std::cout << "press enter key to start controllers." << std::endl;
  getchar();
  manager.load_controllers();
  manager.start_controllers();

  std::cout << "press enter key to delete controllers and model." << std::endl;
  getchar();
  if (!manager.stop_controllers()) {
    RCLCPP_ERROR(rclcpp::get_logger("spawn_model"), "fail to stop controllers in gazebo: %s", type_cstr);
    return 1;
  }
  manager.unload_controllers();
  manager.delete_model();

  rclcpp::shutdown();
  return 0;
}
