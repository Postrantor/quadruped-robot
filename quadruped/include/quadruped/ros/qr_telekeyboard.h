/**
 * @brief
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author postrantor@gmail.com
 * @date 2024-08-04 00:28:11
 * @copyright MIT License
 */

#ifndef QR_TELEKEYBOARD_HPP
#define QR_TELEKEYBOARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <termios.h>
#include <unistd.h>
#include <map>
#include <tuple>
#include <string>
#include <vector>

class qrTeleKeyboard : public rclcpp::Node {
public:
  qrTeleKeyboard();

private:
  /**
   * @brief Keep receiving the event from the keyboard. Keep running until input
   *        Ctrl+C.
   */
  void timer_callback();

  /**
   * @brief Keep sending the default Twist message.
   */
  void default_timer_callback();

  /**
   * @brief Get the event from the keyboard and convert it to corresponding char.
   *        For non-blocking keyboard inputs.
   * @node 非阻塞获取键盘输入
   * @return The number of the event's index.
   */
  int getch_nonblocking();

private:
  /**
   * @brief The topic which the converted message to publish to.
   */
  std::string topic_name_ = "/qr_telekeyboard";

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy joy_msg_;
};

#endif  // QR_TELEKEYBOARD_HPP
