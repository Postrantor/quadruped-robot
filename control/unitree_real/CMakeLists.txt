cmake_minimum_required(VERSION 3.5)
project(unitree_real)

#
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

#
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_sdk REQUIRED)
find_package(unitree_msgs REQUIRED)
find_library(LEGGEDLIB unitree_legged_sdk REQUIRED)
find_library(MOTORLIB UnitreeMotorSDK_M80106_Arm64 REQUIRED)

include_directories(include)
set(EXTRA_LIBS
  -pthread
  lcm
  ${MOTORLIB}
  ${LEGGEDLIB}
)
set(DEPENDENCIES
  rclcpp
  unitree_sdk
  unitree_msgs
)

#
set(EXEC_MOTOR example_motor)
add_executable(${EXEC_MOTOR} src/control_motor.cpp)
target_link_libraries(${EXEC_MOTOR} ${EXTRA_LIBS})
ament_target_dependencies(${EXEC_MOTOR} ${DEPENDENCIES})

# for `ros2 run`
install(
  TARGETS ${EXEC_MOTOR}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
# ament_export_libraries()
ament_package()
