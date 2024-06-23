cmake_minimum_required(VERSION 3.5)
project(unitree_legged_control)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(unitree_msgs REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  unitree_msgs
  controller_manager_msgs
  controller_interface
  hardware_interface
)

# include
include_directories(
  include
)

# set(unitree_joint_control_tool_lib "unitree_joint_control_tool_arm64")
set(unitree_joint_control_tool_lib "unitree_joint_control_tool")

# library
add_library(unitree_legged_control SHARED src/joint_controller.cpp)
target_link_libraries(unitree_legged_control ${unitree_joint_control_tool_lib})
ament_target_dependencies(unitree_legged_control ${DEPENDENCIES})

# install
install(
  TARGETS
    unitree_legged_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
