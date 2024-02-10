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

  void init_motor() {
    // cmd_.motor_cmd[i].mode = 0x01;
    // cmd_.motor_cmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
    // cmd_.motor_cmd[i].kp = 0;
    // cmd_.motor_cmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
    // cmd_.motor_cmd[i].kd = 0;
    // cmd_.motor_cmd[i].tau = 0;
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
    cmd_.dq = 6.28 * get_motor_gear_ratio(MotorType::GO_M8010_6);
    cmd_.tau = 0.0;

    state_.motorType = MotorType::GO_M8010_6;

    serial_.send_recv(&cmd_, &state_);

    if (state_.correct == true) {
      std::cout << state_ << std::endl;
    } else {
      RCLCPP_INFO(this->get_logger(), "get motor response error.");
    }

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
      serial_.send_recv(&cmd_, &state_);

      if (state_.correct == true) {
        std::cout << state_ << std::endl;
      } else {
        RCLCPP_INFO(this->get_logger(), "get motor response error.");
      }
    });
    return true;
  }

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
