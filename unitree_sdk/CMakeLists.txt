cmake_minimum_required(VERSION 3.10)
project(unitree_sdk)

# C/CXX
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

#
include_directories(include)
link_directories(lib)

# install
install(
  DIRECTORY lib include
  DESTINATION .
)

# export
ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  unitree_msgs
)

ament_package()
