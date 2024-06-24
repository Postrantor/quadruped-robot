cmake_minimum_required(VERSION 3.5)
project(robot_sim)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadruped REQUIRED)
find_package(backward_ros REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  quadruped
  unitree_msgs
  gazebo_msgs
  std_srvs
)

include_directories(
  include
)

add_executable(robot_sim simulation/example_simulation_2.cpp)
ament_target_dependencies(qr_gazebo ${DEPENDENCIES})
