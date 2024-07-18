cmake_minimum_required(VERSION 3.5)
project(robot_sim)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadruped REQUIRED)
find_package(backward_ros REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# find_package(unitree_legged_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  quadruped
  unitree_msgs
  gazebo_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  tf2_ros
)

include_directories(
  include
)

add_executable(${PROJECT_NAME} simulation/example_simulation_2.cpp)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

ament_package()
