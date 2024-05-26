cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros2_control_demos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(example_diff_drive
  examples/example_diff_drive.cpp
)
ament_target_dependencies(example_diff_drive
  rclcpp
  geometry_msgs
)

install(
  TARGETS example_diff_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
