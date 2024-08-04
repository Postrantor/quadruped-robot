/**
 * @brief
 * @date 2024-07-29 20:49:03
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"

using namespace std::chrono_literals;

class PeriodicPublisher : public rclcpp::Node {
public:
  PeriodicPublisher() : Node("periodic_publisher") {
    publisher_calf_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FR_calf_controller/desired_state", 10);
    publisher_hip_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FR_hip_controller/desired_state", 10);
    publisher_thigh_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FR_thigh_controller/desired_state", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&PeriodicPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto cmd = unitree_msgs::msg::MotorCmd();
    cmd.hex_len = 17;
    cmd.id = 0;
    cmd.mode = 10;
    cmd.q = 0.15;
    cmd.k_q = 0.0;
    cmd.dq = 0.4;
    cmd.k_dq = 0.0;
    cmd.tau = 0.2;

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '%s'" << cmd.dq);
    publisher_calf_->publish(cmd);
    publisher_hip_->publish(cmd);
    publisher_thigh_->publish(cmd);
  }

  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_calf_;
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_hip_;
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_thigh_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeriodicPublisher>());
  rclcpp::shutdown();

  return 0;
}
