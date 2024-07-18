/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TOOLS_H
#define QR_TOOLS_H

#include <limits.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <string>

#include "quadruped/utils/qr_print.hpp"
#include "quadruped/utils/qr_visualization.h"

namespace robotics {

namespace utils {

/**
 * @brief Query the path of the executable file.
 */
std::string GetExePath();

/**
 * @brief Refer to https://answers.ros.org/question/63491/keyboard-key-pressed
 */
int getch1();

}  // Namespace utils
}  // Namespace robotics

#ifdef _useros
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName = "quadruped-robot/");
#else
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName = "quadruped-robot/");
#endif

#endif  // QR_TOOLS_H
