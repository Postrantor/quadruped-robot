cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_legged_sdk REQUIRED)
find_package(unitree_actuator_sdk REQUIRED)

include_directories(include)
link_directories(${CMAKE_SOURCE_DIR}/lib/cpp)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.a lcm)
set(DEPENDENCY_LIST
    rclcpp
    ros2_unitree_legged_msgs
    unitree_legged_sdk
    unitree_actuator_sdk
    unitree_go
    unitree_api
    rclcpp
    std_msgs
    rosbag2_cpp
)

# src/common/motor_crc.cpp
add_executable(example_ros2_motor src/control_motor_by_ros2.cpp)
target_link_libraries(example_ros2_motor ${EXTRA_LIBS})
ament_target_dependencies(example_ros2_motor ${DEPENDENCY_LIST})

# add_executable(ros2_walk_example src/ros2_walk_example.cpp)
# target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
# ament_target_dependencies(ros2_walk_example ${DEPENDENCY_LIST})

# add_executable(ros2_position_example src/ros2_position_example.cpp)
# target_link_libraries(ros2_position_example ${EXTRA_LIBS})
# ament_target_dependencies(ros2_position_example ${DEPENDENCY_LIST})

install(
  TARGETS
    # ros2_walk_example
    # ros2_position_example
    example_ros2_motor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
# ament_export_libraries(unitree_legged_real)
ament_package()
