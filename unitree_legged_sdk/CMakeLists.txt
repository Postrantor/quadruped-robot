cmake_minimum_required(VERSION 3.10)
project(unitree_legged_sdk)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include_directories(include)
link_directories(lib)

# install
install(
  DIRECTORY lib include
  DESTINATION .
)

# export
ament_export_include_directories(include)

ament_package()
