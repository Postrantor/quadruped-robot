cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(ARCH arm64)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.a lcm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_legged_sdk REQUIRED)

include_directories(include)
link_directories(${CMAKE_SOURCE_DIR}/lib/cpp)

# add_executable(ros2_udp src/ros2_udp.cpp)
# target_link_libraries(ros2_udp ${EXTRA_LIBS})
# ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example
  rclcpp
  ros2_unitree_legged_msgs
  unitree_legged_sdk)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example
  rclcpp
  ros2_unitree_legged_msgs
  unitree_legged_sdk)

install(
  TARGETS
    # ros2_udp
    ros2_walk_example
    ros2_position_example
  DESTINATION lib/${PROJECT_NAME})

ament_package()
