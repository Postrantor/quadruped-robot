cmake_minimum_required(VERSION 3.5)
project(xpp_vis)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(xpp_states REQUIRED)
find_package(xpp_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${robot_state_publisher_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${xpp_states_INCLUDE_DIRS}
  ${xpp_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/urdf_visualizer.cc
  src/ros_msg_helper.cc
  src/cartesian_joint_converter.cc
  src/rviz_robot_builder.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  kdl_parser
  robot_state_publisher
  visualization_msgs
  xpp_states
  xpp_msgs
)

add_executable(rviz_marker_node src/exe/rviz_marker_node.cc)
target_link_libraries(rviz_marker_node
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} rviz_marker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
