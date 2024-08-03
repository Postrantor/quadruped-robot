/**
 * @brief
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @author postrantor@gmail.com
 * @date 2024-08-04 00:28:11
 * @copyright MIT License
 */

#include "quadruped/ros/qr_telekeyboard.h"

qrTeleKeyboard::qrTeleKeyboard() : Node("qr_telekeyboard") {
  joy_msg_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  joy_msg_.axes = {0, 0, 0, 0, 0, 0, 0, 0};

  pub_ = this->create_publisher<sensor_msgs::msg::Joy>(topic_name_, 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&qrTeleKeyboard::timer_callback, this));
}

void qrTeleKeyboard::timer_callback() {
  char key = getch_nonblocking();

  std::map<char, int> buttons_binding{{'k', 0}, {'l', 1}, {'j', 2}, {'i', 3}, {'o', 4}, {'u', 5}};
  std::map<char, std::tuple<int, float>> rockers_binding{{'w', {4, 0.8}},  {'a', {3, 0.8}}, {'s', {4, -0.8}},
                                                         {'d', {3, -0.8}}, {'q', {0, 1.0}}, {'e', {0, -1.0}}};

  // 如果有按键按下，则更新joy_msg_
  if (buttons_binding.count(key) == 1) {
    joy_msg_.buttons[buttons_binding[key]] = 1;
  } else if (rockers_binding.count(key) == 1) {
    joy_msg_.axes[std::get<0>(rockers_binding[key])] = std::get<1>(rockers_binding[key]);
  } else {
    // 如果没有按键按下，则发送0值
    joy_msg_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    joy_msg_.axes = {0, 0, 0, 0, 0, 0, 0, 0};
  }

  if (key == '\x03') {  // Ctrl+C
    RCLCPP_INFO(this->get_logger(), "Break from keyboard.");
    rclcpp::shutdown();
    return;
  }

  pub_->publish(joy_msg_);
}

int qrTeleKeyboard::getch_nonblocking() {
  int ch = -1;
  struct termios oldt, newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // Set non-blocking mode
  struct timeval tv;
  fd_set fds;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);

  if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
    ch = getchar();
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}
