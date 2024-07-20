cmake_minimum_required(VERSION 3.5)
project(xpp_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    # msg/RobotStateCartesianTrajectory.msg
    # msg/RobotStateCartesian.msg
    msg/RobotStateJoint.msg
    msg/RobotParameters.msg
    msg/TerrainInfo.msg
    msg/StateLin3d.msg
    msg/State6d.msg
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  ADD_LINTER_TESTS
)

ament_package()
