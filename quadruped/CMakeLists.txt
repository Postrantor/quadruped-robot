cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# set c++ standard and build type
set(CMAKE_BUILD_TYPE "Debug") # "RELEASE", "Debug"
set(CMAKE_CXX_FLAGS "-O3 -DEIGEN_NO_DEBUG -fopenmp")
add_compile_options(-D_cplusplus -D_useros -fPIC)
# add_compile_options(-Wall -Wextra -Wpedantic -Werror)

option(USE_BLAS "USE MKL BLAS" OFF)
option(USE_SIM "USE Simulation" ON)

if(NOT ${USE_SIM})
  message("[quadruped] not use sim")
  add_definitions(-DNOT_USE_SIM)
endif()

# ros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(xpp_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(unitree_legged_sdk REQUIRED) # A1 & AligenGo
find_package(deeprobotics_legged_sdk REQUIRED)
find_package(tiny_ekf REQUIRED)
find_package(MITAMD REQUIRED)
find_package(tinynurbs REQUIRED)
find_package(quadprog REQUIRED)
find_package(qpOASES REQUIRED)
find_package(lcm REQUIRED)
find_package(matplotlib_cpp REQUIRED)

# to use blas acceleration, uncomment the following lines.
if(USE_BLAS)
  message("[quadruped] using blas")
  find_package(BLAS REQUIRED)
  set(BLA_VENDOR Intel10_64lp)
  add_definitions(-DEIGEN_USE_MKL_ALL)
endif()

include_directories(
  ${unitree_legged_sdk_INCLUDE_DIRS}
  ${matplotlib_cpp_INCLUDE_DIRS}
  include
)

if(CMAKE_CROSSCOMPILING)
  message("[quadruped] crosscompile arm target")
  set(UNITREE_SDK_LIB ${unitree_legged_sdk_DIR}/../../../lib/libunitree_legged_sdk_arm64.so)
else()
  message("[quadruped] compile x86 target")
  set(UNITREE_SDK_LIB ${unitree_legged_sdk_DIR}/../../../lib/libunitree_legged_sdk_amd64.so)
endif()

# add source files
file(GLOB_RECURSE sources
  src/*.cpp
  src/*/*.cpp
  src/*/*/*.cpp
)

if(NOT ${USE_SIM})
  list(
    REMOVE_ITEM
    "include/quadruped/robots/qr_robot_sim.h"
    "include/quadruped/robots/qr_a1_sim.h")
  set(filtered_sources
    ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_sim.cpp
    ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_a1_sim.cpp)
  list(REMOVE_ITEM sources ${filtered_sources})
endif()

# create library
add_library(${PROJECT_NAME} SHARED ${sources})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${UNITREE_SDK_LIB}
  lcm # needed by unitree_legged_sdk
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  tiny_ekf
  quadprog
  tinynurbs
  qpOASES
  MITAMD
  deeprobotics_legged_sdk
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  yaml_cpp_vendor
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  xpp_msgs
  gazebo_msgs
  unitree_msgs
  tf2_ros
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include config
  DESTINATION .
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  yaml_cpp_vendor
  unitree_legged_sdk
  deeprobotics_legged_sdk
  tiny_ekf
  quadprog
  qpOASES
  tinynurbs
  lcm
)

ament_package()
