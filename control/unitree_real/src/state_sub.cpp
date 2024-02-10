/**
 * @brief
 * @date 2024-01-28
 * @copyright Copyright (c) 2024
 */

#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/LowCmd.hpp"
#include "unitree_msgs/LowState.hpp"
#include "unitree_msgs/HighCmd.hpp"
#include "unitree_msgs/HighState.hpp"
#include "unitree_msgs/MotorCmd.hpp"
#include "unitree_msgs/MotorState.hpp"
#include "unitree_msgs/BmsCmd.hpp"
#include "unitree_msgs/BmsState.hpp"
#include "unitree_msgs/IMU.hpp"

#include "sdk/unitree_sdk.h"

void high_state_callback(const unitree_msgs::HighState::ConstPtr &msg) {
  printf("yaw = %f\n", msg->imu.rpy[2]);
}

void low_state_callback(const unitree_msgs::LowState::ConstPtr &msg) {
  printf("FR_2_pos = %f\n", msg->motorState[unitree_sdk::FR_2].q);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("node_high_state_sub");

  unitree_msgs::HighState high_state_ros;
  rclcpp::Subscriber high_sub = nh.subscribe("high_state", 1, high_state_callback);
  rclcpp::Subscriber low_sub = nh.subscribe("low_state", 1, low_state_callback);

  rclcpp::spin(nh);
  return 0;
}