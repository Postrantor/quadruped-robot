/**
 * @file twist_sub.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>
#include <pthread.h>

#include "convert.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>

class Custom {
public:
  UNITREE_LEGGED_SDK::UDP low_udp;
  UNITREE_LEGGED_SDK::LowCmd low_cmd = {0};
  UNITREE_LEGGED_SDK::LowState low_state = {0};

public:
  Custom() : low_udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8091, "192.168.123.10", 8007) {
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
ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;
long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

  custom.high_cmd = rosMsg2Cmd(msg);
  printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
  printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
  printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

  unitree_legged_msgs::HighState high_state_ros = state2rosMsg(custom.high_state);
  pub_high.publish(high_state_ros);

  printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "twist_sub");
  ros::NodeHandle nh;

  pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
  sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

  UNITREE_LEGGED_SDK::LoopFunc loop_udpSend(
      "high_udp_send",  //
      0.002,            //
      3,                //
      boost::bind(&Custom::highUdpSend, &custom));
  UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv(
      "high_udp_recv",  //
      0.002,            //
      3,                //
      boost::bind(&Custom::highUdpRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();

  ros::spin();
  return 0;
}
