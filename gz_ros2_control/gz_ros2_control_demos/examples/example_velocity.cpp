/**
 * @brief
 * @date 2024-03-04
 * @copyright Copyright (c) 2024
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("velocity_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = 1;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = -1;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = 0;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  rclcpp::shutdown();

  return 0;
}
