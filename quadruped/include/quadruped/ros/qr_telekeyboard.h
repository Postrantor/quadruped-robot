/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TELEKEYBOARD_H
#define QR_TELEKEYBOARD_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <vector>

/**
 * @brief qrTeleKeyboard 类用于将键盘接收到的消息转换为 Twist 消息，以便 qrVelocityParamReceiver 接收。
 */
class qrTeleKeyboard {
public:
  /**
   * @brief qrTeleKeyboard 的构造函数。
   * @param nh 该类创建的 ROS 节点。
   */
  qrTeleKeyboard(const rclcpp::Node::SharedPtr& nhIn);

  /**
   * @brief qrTeleKeyboard 的默认析构函数。
   */
  ~qrTeleKeyboard() {}

  /**
   * @brief 从键盘获取事件并将其转换为相应的字符。
   *        用于非阻塞键盘输入。
   * @return 事件的索引号。
   */
  int getch();

  /**
   * @brief 不断地从键盘接收事件，直到输入 Ctrl+C。
   */
  void run();

  /**
   * @brief 不断地发送默认的 Twist 消息。
   */
  void run_default();

  /**
   * @brief 键盘是否停止接收事件。
   */
  bool finish;

  /**
   * @brief 用于同步 run 和 run_default 线程的互斥锁。
   */
  bool mutex;

private:
  /**
   * @brief 转换后的消息要发布到的主题。
   */
  std::string cmdTopic = "/joy";

  /**
   * @brief 该类创建的 ROS 节点。
   */
  rclcpp::Node::SharedPtr& nh;
};

#endif  // QR_TELEKEYBOARD_H
