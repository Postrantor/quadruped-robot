/**
 * @brief use ros2 to send low-level motor commands of unitree go2 robot
 * @copyright
 */

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/msg/low_cmd.hpp"
#include "unitree_msgs/msg/bms_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"

#include "common/convert.h"
#include "serial/serial_port.hpp"
#include "motor/motor_control.hpp"
// #include "sdk/unitree_sdk.h"

using namespace std::chrono_literals;

class low_level_motor_cmd_sender : public rclcpp::Node {
public:
  low_level_motor_cmd_sender() : Node("low_level_motor_cmd_sender"), serial_("/dev/ttyUSB0") {
    pub_state_ = this->create_publisher<unitree_msgs::msg::MotorState>("/motor_state", 1);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&low_level_motor_cmd_sender::timer_callback, this));
    // init_motor();
    add_shutdown_callback();
  }

  void init_motor() {
    // motor_cmd_.motor_cmd[i].mode = 0x01;
    // motor_cmd_.motor_cmd[i].q = unitree_sdk::PosStopF;
    // motor_cmd_.motor_cmd[i].kp = 0;
    // motor_cmd_.motor_cmd[i].dq = unitree_sdk::VelStopF;
    // motor_cmd_.motor_cmd[i].kd = 0;
    // motor_cmd_.motor_cmd[i].tau = 0;
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "timer callback: %ld", count_++);

    // sub?
    motor_cmd_.motorType = MotorType::GO_M8010_6;
    motor_cmd_.mode = set_mode(MotorMode::FOC);
    motor_cmd_.id = 0;
    motor_cmd_.k_q = 0.0;
    motor_cmd_.k_dq = 0.05;
    motor_cmd_.q = 0.0;
    motor_cmd_.dq = 6.28 * get_gear_ratio(MotorType::GO_M8010_6);
    motor_cmd_.tau = 0.0;
    std::cout << motor_cmd_ << std::endl;

    motor_state_.motorType = MotorType::GO_M8010_6;

    serial_.send_recv(&motor_cmd_, &motor_state_);

    if (!get_crc(motor_state_)) {
      RCLCPP_INFO(this->get_logger(), "get motor response error.");
    }
    std::cout << motor_state_ << std::endl;
    pub_state_->publish(unitree_sdk::motor2msg(motor_state_));
  }

  bool add_shutdown_callback() {
    auto context = rclcpp::contexts::get_global_default_context();
    if (!context->is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "context is invaild.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "register on_shutdown_callback.");
    context->add_on_shutdown_callback([this]() {
      motor_cmd_.motorType = MotorType::GO_M8010_6;
      motor_cmd_.mode = set_mode(MotorMode::BRAKE);
      motor_cmd_.id = 0;
      motor_state_.motorType = MotorType::GO_M8010_6;
      serial_.send_recv(&motor_cmd_, &motor_state_);

      if (motor_state_.correct == true) {
        std::cout << motor_state_ << std::endl;
      } else {
        RCLCPP_INFO(this->get_logger(), "get motor response error.");
      }
    });
    return true;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_msgs::msg::MotorState>::SharedPtr pub_state_;

  std::size_t count_{0};
  MotorCmd motor_cmd_;
  MotorData motor_state_;
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
  auto node = std::make_shared<low_level_motor_cmd_sender>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
