/**
 * @file ros2_position_example.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <cmath>

#include "convert.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv) {
  std::cout << "communication level is set to low-level." << std::endl
            << "warning: make sure the robot is hung up." << std::endl
            << "press enter to continue..." << std::endl;
  std::cin.ignore();

  // ros init
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("node_ros2_postition_example");
  auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);

  rclcpp::WallRate loop_rate(500);
  // FIXME(@zhiqi.jia) use count > 10?
  bool initiated_flag = false;  // initiate need time
  int count = 0;
  long motion_time = 0;

  // set motor init parameters
  ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;
  low_cmd_ros.head[0] = 0xFE;  // reserve?
  low_cmd_ros.head[1] = 0xEF;  // reserve?
  low_cmd_ros.level_flag = LOWLEVEL;
  for (int i = 0; i < 12; i++) {
    // motor target mode
    // - servo : 0x0A
    // - damping : 0x00.
    low_cmd_ros.motor_cmd[i].mode = 0x0A;  // servo mode(PMSM)

    // FIXME(@zhiqi.jia) ?
    // 禁止位置环((2.146E+9f), define in sdk common.h)
    low_cmd_ros.motor_cmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
    // 禁止速度环((16000.0f), above on)
    low_cmd_ros.motor_cmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
    // use force loop
    low_cmd_ros.motor_cmd[i].tau = 0;
    low_cmd_ros.motor_cmd[i].kp = 0;
    low_cmd_ros.motor_cmd[i].kd = 0;
  }

  while (rclcpp::ok()) {
    if (initiated_flag == true) {
      motion_time += 2;

      low_cmd_ros.motor_cmd[FR_0].tau = -0.65f;
      low_cmd_ros.motor_cmd[FL_0].tau = +0.65f;
      low_cmd_ros.motor_cmd[RR_0].tau = -0.65f;
      low_cmd_ros.motor_cmd[RL_0].tau = +0.65f;

      low_cmd_ros.motor_cmd[FR_2].q = -M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * motion_time * 1e-3);
      low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
      low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
      low_cmd_ros.motor_cmd[FR_2].kd = 1.0;

      low_cmd_ros.motor_cmd[FR_0].q = 0.0;
      low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
      low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
      low_cmd_ros.motor_cmd[FR_0].kd = 1.0;

      low_cmd_ros.motor_cmd[FR_1].q = 0.0;
      low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
      low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
      low_cmd_ros.motor_cmd[FR_1].kd = 1.0;
    }
    count++;
    if (count > 10) {
      count = 10;
      initiated_flag = true;
    }

    pub->publish(low_cmd_ros);

    // FIXME(@zhiqi.jia) use timer?
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
