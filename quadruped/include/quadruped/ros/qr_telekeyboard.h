/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TELEKEYBOARD_H
#define QR_TELEKEYBOARD_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <map>
#include <vector>

/**
 * @brief Class qrTeleKeyboard is used to convert the message received from the keyboard
 *        to Twist message which could be received by qrVelocityParamReceiver.
 */
class qrTeleKeyboard {
public:
  /**
   * @brief Constructor of qrTeleKeyboard.
   * @param nh The ros node which this class create from.
   */
  qrTeleKeyboard(ros::NodeHandle &nhIn);

  /**
   * @brief Default destructor of qrJoy2Twist.
   */
  ~qrTeleKeyboard() {}

  /**
   * @brief Get the event from the keyboard and convert it to corresponding char.
   *        For non-blocking keyboard inputs.
   * @return The number of the event's index.
   */
  int getch();

  /**
   * @brief Keep receiving the event from the keyboard. Keep running until input
   *        Ctrl+C.
   */
  void run();

  /**
   * @brief Keep sending the default Twist message.
   */
  void run_default();

  /**
   * @brief If the keyboard stop receiving event.
   */
  bool finish;

  /**
   * @brief mutex used to synchronize run and run_default thread.
   */
  bool mutex;

private:
  /**
   * @brief The topic which the converted message to publish to.
   */
  std::string cmdTopic = "/joy";

  /**
   * @brief The ros node which this class create from.
   */
  ros::NodeHandle &nh;
};

#endif  // QR_TELEKEYBOARD_H
