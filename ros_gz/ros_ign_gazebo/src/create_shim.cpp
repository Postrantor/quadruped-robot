/**
 * @brief
 * @date 2024-03-04
 * @copyright Copyright (c) 2024
 */

// Shim to redirect "ros_ign_bridge parameter_bridge" call to "ros_gz_sim parameter_bridge"

#include <sstream>
#include <iostream>
#include <stdlib.h>

#include <ament_index_cpp/get_package_prefix.hpp>

int main(int argc, char* argv[]) {
  std::stringstream cli_call;

  cli_call << ament_index_cpp::get_package_prefix("ros_gz_sim") << "/lib/ros_gz_sim/create";

  if (argc > 1) {
    for (int i = 1; i < argc; i++) cli_call << " " << argv[i];
  }

  std::cerr << "[ros_ign_gazebo] is deprecated! "
            << "Redirecting to use [ros_gz_sim] instead!" << std::endl
            << std::endl;
  system(cli_call.str().c_str());

  return 0;
}
