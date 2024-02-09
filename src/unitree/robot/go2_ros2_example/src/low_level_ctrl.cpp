/**
 * @brief use ros2 to send low-level motor commands of unitree go2 robot
 */

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

#include "common/motor_crc.h"

using namespace std::chrono_literals;

// create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {
    // the cmd_pub_ is set to subscribe "/lowcmd" topic
    cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    // the timer is set to 200hz, and bind to low_level_cmd_sender::timer_callback function
    timer_ = this->create_wall_timer(5ms, std::bind(&low_level_cmd_sender::timer_callback, this));
    // initialize lowcmd
    init_cmd();
    // running time count
    // t = 0;
  }

private:
  void timer_callback() {
    // test code here
    // t += 0.02;

    // toque control, set rl_2 toque
    cmd_.motor_cmd[RL_2].q = PosStopF;  // Set to stop position(rad)
    cmd_.motor_cmd[RL_2].kp = 0;
    cmd_.motor_cmd[RL_2].dq = VelStopF;  // Set to stop angular velocity(rad/s)
    cmd_.motor_cmd[RL_2].kd = 0;
    cmd_.motor_cmd[RL_2].tau = 1;  // target toque is set to 1N.m

    // poinstion(rad) control, set rl_0 rad
    cmd_.motor_cmd[RL_0].q = 0;    // Taregt angular(rad)
    cmd_.motor_cmd[RL_0].kp = 10;  // Poinstion(rad) control kp gain
    cmd_.motor_cmd[RL_0].dq = 0;   // Taregt angular velocity(rad/ss)
    cmd_.motor_cmd[RL_0].kd = 1;   // Poinstion(rad) control kd gain
    cmd_.motor_cmd[RL_0].tau = 0;  // Feedforward toque 1N.m

    get_crc(cmd_);            // check motor cmd crc
    cmd_pub_->publish(cmd_);  // Publish lowcmd message
  }

  void init_cmd() {
    for (int i = 0; i < 20; i++) {
      cmd_.motor_cmd[i].mode = 0x01;  // set toque mode, 0x00 is passive mode
      cmd_.motor_cmd[i].q = PosStopF;
      cmd_.motor_cmd[i].kp = 0;
      cmd_.motor_cmd[i].dq = VelStopF;
      cmd_.motor_cmd[i].kd = 0;
      cmd_.motor_cmd[i].tau = 0;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_pub_;
  unitree_go::msg::LowCmd cmd_;
  // double t;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // rclcpp::TimerBase::SharedPtr timer_;
  auto node = std::make_shared<low_level_cmd_sender>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
