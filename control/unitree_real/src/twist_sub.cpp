/**
 * @brief
 * @date 2024-01-28
 * @copyright Copyright (c) 2024
 */

#include <chrono>
#include <pthread.h>

#include "convert.h"
#include "sdk/unitree_sdk.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Twist.hpp"
#include "unitree_msgs/HighCmd.hpp"
#include "unitree_msgs/HighState.hpp"
#include "unitree_msgs/LowCmd.hpp"
#include "unitree_msgs/LowState.hpp"

class Custom {
public:
  unitree_sdk::UDP low_udp;
  unitree_sdk::LowCmd low_cmd = {0};
  unitree_sdk::LowState low_state = {0};

public:
  Custom() : low_udp(unitree_sdk::LOWLEVEL, 8091, "192.168.123.10", 8007) {
    low_udp.InitCmdData(low_cmd);
  }
  void lowUdpSend() {
    low_udp.SetSend(low_cmd);
    low_udp.Send();
  }
  void lowUdpRecv() {
    low_udp.Recv();
    low_udp.GetRecv(low_state);
  }
};

Custom custom;
rclcpp::Subscriber sub_cmd_vel;
rclcpp::Publisher pub_high;
long cmd_vel_count = 0;

void cmdvel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  printf("cmdvel_callback is running!\t%ld\n", cmd_vel_count);

  custom.high_cmd = msg2motor(msg);
  printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
  printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
  printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

  unitree_msgs::HighState high_state_ros = motor2msg(custom.high_state);
  pub_high.publish(high_state_ros);

  printf("cmdvel_callback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, "twist_sub");
  rclcpp::NodeHandle nh;

  pub_high = nh.advertise<unitree_msgs::HighState>("high_state", 1);
  sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdvel_callback);

  unitree_sdk::LoopFunc loop_udpSend(
      "high_udp_send",  //
      0.002,            //
      3,                //
      boost::bind(&Custom::highUdpSend, &custom));
  unitree_sdk::LoopFunc loop_udpRecv(
      "high_udp_recv",  //
      0.002,            //
      3,                //
      boost::bind(&Custom::highUdpRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();

  rclcpp::spin();
  return 0;
}
