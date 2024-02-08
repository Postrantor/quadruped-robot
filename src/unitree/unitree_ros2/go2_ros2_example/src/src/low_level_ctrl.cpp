/**
 * @brief this example demonstrates how to use ros2 to send low-level motor commands of unitree go2
 * robot
 */

#include "rclcpp/rclcpp.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

#include "common/motor_crc.h"

// create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {
    // the cmd_puber is set to subscribe "/lowcmd" topic
    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // the timer is set to 200hz, and bind to low_level_cmd_sender::timer_callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),  //
        std::bind(&low_level_cmd_sender::timer_callback, this));

    // initialize lowcmd
    init_cmd();

    // running time count
    t = 0;
  }

private:
  void timer_callback() {
    // test code here
    t += 0.02;

    // toque control, set rl_2 toque
    cmd_msg.motor_cmd[RL_2].q = PosStopF;  // Set to stop position(rad)
    cmd_msg.motor_cmd[RL_2].kp = 0;
    cmd_msg.motor_cmd[RL_2].dq = VelStopF;  // Set to stop angular velocity(rad/s)
    cmd_msg.motor_cmd[RL_2].kd = 0;
    cmd_msg.motor_cmd[RL_2].tau = 1;  // target toque is set to 1N.m

    // poinstion(rad) control, set rl_0 rad
    cmd_msg.motor_cmd[RL_0].q = 0;    // Taregt angular(rad)
    cmd_msg.motor_cmd[RL_0].kp = 10;  // Poinstion(rad) control kp gain
    cmd_msg.motor_cmd[RL_0].dq = 0;   // Taregt angular velocity(rad/ss)
    cmd_msg.motor_cmd[RL_0].kd = 1;   // Poinstion(rad) control kd gain
    cmd_msg.motor_cmd[RL_0].tau = 0;  // Feedforward toque 1N.m

    get_crc(cmd_msg);             // check motor cmd crc
    cmd_puber->publish(cmd_msg);  // Publish lowcmd message
  }

  void init_cmd() {
    for (int i = 0; i < 20; i++) {
      cmd_msg.motor_cmd[i].mode = 0x01;  // set toque mode, 0x00 is passive mode
      cmd_msg.motor_cmd[i].q = PosStopF;
      cmd_msg.motor_cmd[i].kp = 0;
      cmd_msg.motor_cmd[i].dq = VelStopF;
      cmd_msg.motor_cmd[i].kd = 0;
      cmd_msg.motor_cmd[i].tau = 0;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
  unitree_go::msg::LowCmd cmd_msg;
  double t;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // create a timer callback object to send cmd in time intervals
  rclcpp::TimerBase::SharedPtr timer_;
  // create a ros2 node and make share with low_level_cmd_sender class
  auto node = std::make_shared<low_level_cmd_sender>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
