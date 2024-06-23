/**
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <termios.h>
#include <unistd.h>
#include <map>
#include <tuple>
#include <iostream>

class qrTeleKeyboard : public rclcpp::Node {
public:
  qrTeleKeyboard() : Node("qr_telekeyboard"), finish(false), mutex(false) {
    pub = this->create_publisher<sensor_msgs::msg::Joy>(cmdTopic, 1);
  }

  int getch(void) {
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
  }

  void run() {
    auto joy_msg = sensor_msgs::msg::Joy();
    char key = ' ';
    // initialize joy cmd
    joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    joy_msg.axes = {0, 0, 0, 0, 0, 0, 0, 0};

    // joy's A X B Y Rb
    std::map<char, int> buttons_binding;
    buttons_binding['k'] = 0;  // joy A
    buttons_binding['l'] = 1;  // joy B
    buttons_binding['j'] = 2;  // joy X
    buttons_binding['i'] = 3;  // joy Y
    buttons_binding['o'] = 4;  // joy RB
    buttons_binding['u'] = 5;  // joy RL

    std::map<char, std::tuple<int, float>> rockers_binding;
    rockers_binding['w'] = {4, 0.8};
    rockers_binding['a'] = {3, 0.8};
    rockers_binding['s'] = {4, -0.8};
    rockers_binding['d'] = {3, -0.8};
    rockers_binding['q'] = {0, 1.0};
    rockers_binding['e'] = {0, -1.0};

    while (rclcpp::ok()) {
      key = getch();
      mutex = true;

      if (buttons_binding.count(key) == 1) {
        joy_msg.buttons[buttons_binding[key]] = 1;
      } else if (rockers_binding.count(key) == 1) {
        joy_msg.axes[std::get<0>(rockers_binding[key])] = std::get<1>(rockers_binding[key]);
      }

      // receive Ctrl+C
      if (key == '\x03') {
        std::cout << "Break from keyboard." << std::endl;
        finish = true;
        break;
      }

      pub->publish(joy_msg);

      // clear the joy_msg
      joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      joy_msg.axes = {0, 0, 0, 0, 0, 0, 0, 0};

      rclcpp::spin_some(this->get_node_base_interface());
      mutex = false;
    }
  }

  void run_default() {
    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    joy_msg.axes = {0, 0, 0, 0, 0, 0, 0, 0};
    while (rclcpp::ok()) {
      if (finish) break;
      if (!mutex) pub->publish(joy_msg);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub;
  bool finish;
  bool mutex;
  const std::string cmdTopic = "cmd_topic";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<qrTeleKeyboard>();
  std::thread run_thread(&qrTeleKeyboard::run, node);
  node->run_default();
  run_thread.join();
  rclcpp::shutdown();
  return 0;
}
