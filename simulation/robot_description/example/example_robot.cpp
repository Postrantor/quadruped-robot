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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class PeriodicPublisher : public rclcpp::Node {
public:
  PeriodicPublisher() : Node("periodic_publisher") {
    // publisher_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FL_thigh_controller/desired_state", 10);
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/FL_thigh_controller/desired_state", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&PeriodicPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // auto cmd = unitree_msgs::msg::MotorCmd();
    // cmd.hex_len = 17;
    // cmd.id = 0;
    // cmd.mode = 10;
    // cmd.tau = 0.2;
    // cmd.dq = 0.2;
    // cmd.q = 0.15;
    // cmd.k_q = 0.0;
    // cmd.k_dq = 0.0;

    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.twist.linear.x = 0.3;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '%s'" << cmd.twist.linear.x);
    publisher_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeriodicPublisher>());
  rclcpp::shutdown();

  return 0;
}
