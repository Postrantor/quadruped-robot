/**
 * @brief
 * @date 2024-08-03 23:45:03
 * @copyright MIT License
 */

#include "rclcpp/rclcpp.hpp"

#include "quadruped/ros/qr_telekeyboard.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto keyboard = std::make_shared<qrTeleKeyboard>();
  RCLCPP_INFO_STREAM(
      keyboard->get_logger(), "User can use:\n"
                                  << "'l'             to torque-stance\n"
                                  << "'j'             to change the gait\n"
                                  << "'u'             to position-stand up/down\n"
                                  << "'i'             to exit after sit down\n\n"
                                  << "'w' 'a' 's' 'd' to control the robot's movement\n"
                                  << "'q' 'e'         to control the robot's rotation\n\n"
                                  << "'ctrl+c'        to exit keyboard_control");

  rclcpp::spin(keyboard);
  rclcpp::shutdown();

  return 0;
}
