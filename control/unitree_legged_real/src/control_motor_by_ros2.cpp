/**
 * @brief use ros2 to send low-level motor commands of unitree go2 robot
 * @copyright
 */

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

#include "convert.h"
#include "serial/serial_port.hpp"
#include "motor/motor_control.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std::chrono_literals;

// create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender"), serial_("/dev/ttyUSB0") {
    cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/cmd_motor", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&low_level_cmd_sender::timer_callback, this));
    // init_motor();
    add_shutdown_callback();
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "timer callback: %ld", count_++);

    cmd_.motorType = MotorType::GO_M8010_6;
    cmd_.mode = set_motor_mode(MotorMode::FOC);
    cmd_.id = 0;
    cmd_.k_q = 0.0;
    cmd_.k_dq = 0.05;
    cmd_.q = 0.0;
    cmd_.dq = 6.28 * GearRatio;
    cmd_.tau = 0.0;

    state_.motorType = MotorType::GO_M8010_6;

    serial_.sendRecv(&cmd_, &state_);

    if (state_.correct == true) {
      std::cout << "motor.q: " << state_.q << " rad" << std::endl;
      std::cout << "motor.dq: " << state_.dq << " rad/s" << std::endl;
      std::cout << "motor.tau: " << state_.tau << " N·m" << std::endl;
      std::cout << "motor.temp: " << state_.temp << " ℃" << std::endl;
      std::cout << "motor.error: " << state_.error << std::endl;
      std::cout << std::endl;
    } else {
      RCLCPP_INFO(this->get_logger(), "get motor response error.");
    }

    // toque control, set rl_2 toque
    // cmd_.motor_cmd[RL_2].q = PosStopF;  // Set to stop position(rad)
    // cmd_.motor_cmd[RL_2].kp = 0;
    // cmd_.motor_cmd[RL_2].dq = VelStopF;  // Set to stop angular velocity(rad/s)
    // cmd_.motor_cmd[RL_2].kd = 0;
    // cmd_.motor_cmd[RL_2].tau = 1;  // target toque is set to 1N.m

    // get_crc(cmd_);  // check motor cmd crc
    // cmd_pub_->publish(cmd);  // publish cmd_low message
  }

  bool add_shutdown_callback() {
    auto context = rclcpp::contexts::get_global_default_context();
    if (!context->is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "context is invaild.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "register on_shutdown_callback.");
    context->add_on_shutdown_callback([this]() {
      cmd_.motorType = MotorType::GO_M8010_6;
      cmd_.mode = set_motor_mode(MotorMode::BRAKE);
      cmd_.id = 0;
      state_.motorType = MotorType::GO_M8010_6;
      serial_.sendRecv(&cmd_, &state_);

      if (state_.correct == true) {
        std::cout << "motor.q: " << state_.q << " rad" << std::endl;
        std::cout << "motor.dq: " << state_.dq << " rad/s" << std::endl;
        std::cout << "motor.tau: " << state_.tau << " N·m" << std::endl;
        std::cout << "motor.temp: " << state_.temp << " ℃" << std::endl;
        std::cout << "motor.error: " << state_.error << std::endl;
        std::cout << std::endl;
      } else {
        RCLCPP_INFO(this->get_logger(), "get motor response error.");
      }
    });
    return true;
  }

  // void init_motor() {
  //   for (int i = 0; i < 20; i++) {
  //     cmd_.motor_cmd[i].mode = 0x01;  // set toque mode, 0x00 is passive mode
  //     cmd_.motor_cmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
  //     cmd_.motor_cmd[i].kp = 0;
  //     cmd_.motor_cmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
  //     cmd_.motor_cmd[i].kd = 0;
  //     cmd_.motor_cmd[i].tau = 0;
  //   }
  // }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_pub_;
  // unitree_go::msg::LowCmd cmd_;

  std::uint64_t count_{0};

  MotorCmd cmd_;
  MotorData state_;
  SerialPort serial_;
};

int main(int argc, char **argv) {
  std::cout << "communication level is set to low-level." << std::endl
            << "warning: make sure the robot is hung up." << std::endl
            << "note: the robot also needs to be set to low-level mode, otherwise it will make "
               "strange noises and this example will not run successfully! "
            << std::endl
            << "press enter to continue..." << std::endl;
  std::cin.ignore();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<low_level_cmd_sender>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
