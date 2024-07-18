/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "quadruped/utils/qr_tools.h"

namespace robotics {

namespace utils {

/**
 * @brief Query the path of the executable file.
 */
std::string GetExePath() {
  char result[PATH_MAX];
  std::size_t count = readlink("/proc/self/exe", result, PATH_MAX);
  return std::string(result, (count > 0) ? count : 0);
}

/**
 * @brief Refer to https://answers.ros.org/question/63491/keyboard-key-pressed
 */
int getch1() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);  // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);  // disable buffering
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

}  // Namespace utils

}  // Namespace robotics

#ifdef _useros
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName) {
  std::string exepath = robotics::utils::GetExePath();
  std::size_t found = exepath.find(homeName);
  std::string homeDir = exepath.substr(0, found) + homeName + "src/quadruped/";
  std::cout << "[USE ROS]: " << homeDir << std::endl;
  return homeDir;
}
#else
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName) {
  std::string exepath = robotics::utils::GetExePath();
  std::size_t found = exepath.find(homeName);
  std::string homeDir = exepath.substr(0, found) + homeName;
  std::cout << "[NO ROS]: " << homeDir << std::endl;
  return homeDir;
}
#endif
