/**
 * @file ros2_walk_example.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include "convert.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

int main(int argc, char **argv) {
  std::cout << "warning: control level is set to high-level." << std::endl
            << "make sure the robot is standing on the ground." << std::endl
            << "press enter to continue..." << std::endl;
  std::cin.ignore();

  // ros init
  rclcpp::init(argc, argv);
  ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
  rclcpp::WallRate loop_rate(500);
  long motion_time = 0;

  auto node = rclcpp::Node::make_shared("node_ros2_walk_example");
  auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

  while (rclcpp::ok()) {
    motion_time += 2;

    // TODO(@zhiqi.jia): 类似于 udp?和电机建立连接？
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;

    high_cmd_ros.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gait_type = 0;
    high_cmd_ros.speed_level = 0;
    high_cmd_ros.foot_raise_height = 0;
    high_cmd_ros.body_height = 0;

    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;

    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yaw_speed = 0.0f;

    high_cmd_ros.reserve = 0;

    if (motion_time > 0 && motion_time < 1000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[0] = -0.3;
    }
    if (motion_time > 1000 && motion_time < 2000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[0] = 0.3;
    }
    if (motion_time > 2000 && motion_time < 3000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[1] = -0.2;
    }
    if (motion_time > 3000 && motion_time < 4000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[1] = 0.2;
    }
    if (motion_time > 4000 && motion_time < 5000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[2] = -0.2;
    }
    if (motion_time > 5000 && motion_time < 6000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.euler[2] = 0.2;
    }
    if (motion_time > 6000 && motion_time < 7000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.body_height = -0.2;
    }
    if (motion_time > 7000 && motion_time < 8000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.body_height = 0.1;
    }
    if (motion_time > 8000 && motion_time < 9000) {
      high_cmd_ros.mode = 1;
      high_cmd_ros.body_height = 0.0;
    }
    if (motion_time > 9000 && motion_time < 11000) {
      high_cmd_ros.mode = 5;
    }
    if (motion_time > 11000 && motion_time < 13000) {
      high_cmd_ros.mode = 6;
    }
    if (motion_time > 13000 && motion_time < 14000) {
      high_cmd_ros.mode = 0;
    }
    if (motion_time > 14000 && motion_time < 18000) {
      high_cmd_ros.mode = 2;
      high_cmd_ros.gait_type = 2;
      high_cmd_ros.velocity[0] = 0.4f;  // -1  ~ +1
      high_cmd_ros.yaw_speed = 2;
      high_cmd_ros.foot_raise_height = 0.1;
      std::cout << "walk" << std::endl;
    }
    if (motion_time > 18000 && motion_time < 20000) {
      high_cmd_ros.mode = 0;
      high_cmd_ros.velocity[0] = 0;
    }
    if (motion_time > 20000 && motion_time < 24000) {
      high_cmd_ros.mode = 2;
      high_cmd_ros.gait_type = 1;
      high_cmd_ros.velocity[0] = 0.2f;  // -1  ~ +1
      high_cmd_ros.body_height = 0.1;
      std::cout << "walk" << std::endl;
    }
    if (motion_time > 24000) {
      high_cmd_ros.mode = 1;
    }

    pub->publish(high_cmd_ros);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
