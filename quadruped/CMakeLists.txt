cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES C CXX)

# Set C++ standard and build type
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE") # Options: "RELEASE", "Debug"
set(CMAKE_CXX_FLAGS "-O3 -DEIGEN_NO_DEBUG -fopenmp ${CMAKE_CXX_FLAGS}")

# Print the CXX flags
message("CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# Add compile options
add_compile_options(-D_cplusplus -D_useros -fPIC)

option(USE_GO1 "WHICH ROBOT" OFF)
option(USE_BLAS "USE MKL BLAS" OFF)
option(USE_SIM "USE Simulation" ON)

if(NOT ${USE_SIM})
  message("not use sim")
  add_definitions(-DNOT_USE_SIM)
endif()

# Set include and link paths
set(includePath)
set(linkPath)

# Determine system architecture
execute_process( # x86_64, amd64, arm64
  COMMAND uname -m
  COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

# Set Unitree SDK library based on architecture
if(ARCHITECTURE STREQUAL "x86_64" OR ARCHITECTURE STREQUAL "amd64")
  set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(ARCHITECTURE STREQUAL "arm64" OR ARCHITECTURE STREQUAL "aarch64")
  set(UNITREE_SDK_LIB "unitree_legged_sdk_arm64.so")
else()
  message(FATAL_ERROR "Unrecognized architecture.")
endif()

list(APPEND includePath "/usr/include/eigen3")

# To use BLAS acceleration, uncomment the following lines.
if(USE_BLAS)
  find_package(BLAS REQUIRED)
  message("Using BLAS")
  set(BLA_VENDOR Intel10_64lp)
  add_definitions(-DEIGEN_USE_MKL_ALL)
endif()

# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)
list(APPEND includePath ${YAML_INCLUDE_DIRS})

# Unitree SDK
if(${USE_GO1})
  set(UNITREE_SDK_GO1_DIR ${PROJECT_SOURCE_DIR}/extern/unitree_legged_sdk_3.5)
  add_subdirectory(${UNITREE_SDK_GO1_DIR})
  list(APPEND includePath "${UNITREE_SDK_GO1_DIR}/include")
  list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/extern/unitree_legged_sdk_3.5)
  list(APPEND linkPath ${UNITREE_SDK_GO1_DIR}/lib)
else() # A1 & AligenGo
  set(UNITREE_SDK_DIR ${PROJECT_SOURCE_DIR}/extern/unitree_legged_sdk)
  add_subdirectory(${UNITREE_SDK_DIR})
  list(APPEND includePath "${UNITREE_SDK_DIR}/include")
  list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/extern/unitree_legged_sdk)
  list(APPEND linkPath ${UNITREE_SDK_DIR}/lib)
endif()

# Deeprobotics SDK
set(DEEPROBOTICS_SDK_DIR ${PROJECT_SOURCE_DIR}/extern/deeprobotics_legged_sdk)
add_subdirectory(${DEEPROBOTICS_SDK_DIR})
list(APPEND includePath ${DEEPROBOTICS_SDK_DIR}/include)
list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/../../devel/lib)

# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/extern/TinyEKF)
list(APPEND includePath "${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")

# matplotlib-cpp
find_package(PythonLibs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
list(APPEND includePath ${PYTHON_INCLUDE_DIRS})
list(APPEND includePath ${PROJECT_SOURCE_DIR}/extern/matplotlib-cpp)

# MIT AMD
add_subdirectory("include/quadruped/utils/amd")

# library TinyNURBS
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/tinynurbs)

# library QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/extern/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
list(APPEND includePath "${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/extern/QuadProgpp)

# library qpOASES
set(qpOASES_DIR ${PROJECT_SOURCE_DIR}/extern/qpOASES)
add_subdirectory(${qpOASES_DIR})
list(APPEND includePath "${qpOASES_DIR}")
list(APPEND includePath "${qpOASES_DIR}/include")

# # library xpp
# set(XPP_LIBS)
# if(${USE_SIM})
#   set(XPP_DIR ${PROJECT_SOURCE_DIR}/../xpp)
#   list(APPEND includePath ${XPP_DIR}/xpp_states/include
#        ${XPP_DIR}/xpp_msgs/include)
#   list(APPEND XPP_LIBS xpp_states xpp_vis)
# endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

list(APPEND includePath include/quadruped src)

# add source files
file(GLOB_RECURSE sources
  src/*.cpp
  src/*/*.cpp
  src/*/*/*.cpp
)

if(NOT ${USE_SIM})
  list(
    REMOVE_ITEM
    includePath
    "include/quadruped/robots/qr_robot_sim.h"
    "include/quadruped/robots/qr_a1_sim.h")
  set(filtered_sources
      ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_sim.cpp
      ${PROJECT_SOURCE_DIR}/src/robots/qr_robot_a1_sim.cpp)
  list(REMOVE_ITEM sources ${filtered_sources})
endif()

# add include files
list(APPEND includePath "config/")
list(APPEND includePath ".")

# create library
add_library(quadruped SHARED ${sources} ${TinyEKF_SRC})
target_include_directories(quadruped PUBLIC ${includePath})
target_link_directories(quadruped PUBLIC ${linkPath})
target_link_libraries(quadruped PUBLIC
  ${PYTHON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${BLAS_LIBRARIES}
  # ${XPP_LIBS}
  ${UNITREE_SDK_LIB}
  deeprobotics_legged_sdk
  lcm
  tinynurbs::tinynurbs
  MITAMD
  quadprog
  qpOASES
)

if(${USE_GO1})
  target_link_libraries(quadruped PUBLIC robot_interface_3.5)
else() # A1 & AligenGo
  target_link_libraries(quadruped PUBLIC robot_interface)
endif()

ament_target_dependencies(quadruped PUBLIC
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  unitree_msgs
  tf2_ros
)

install(
  TARGETS quadruped
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY ${UNITREE_SDK_DIR}/lib/
  DESTINATION lib
)

install(
  DIRECTORY include/quadruped
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION include/config/
)

install(
  DIRECTORY ${UNITREE_SDK_DIR}/include/unitree_legged_sdk
  DESTINATION include/
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/extern/tinynurbs/include/tinynurbs
  DESTINATION include/
)

install(
  DIRECTORY ${DEEPROBOTICS_SDK_DIR}/include/deeprobotics_legged_sdk
  DESTINATION include/
)

install(
  DIRECTORY ${DEEPROBOTICS_SDK_DIR}/include/
  DESTINATION include/
)

install(
  FILES
    ${TinyEKF_DIR}/src/tiny_ekf_struct.h
    ${TinyEKF_DIR}/src/TinyEKF.h
    ${TinyEKF_DIR}/src/tiny_ekf.h
  DESTINATION include/
)

install(
  FILES
    ${QuadProgpp_DIR}/src/Array.hh
    ${QuadProgpp_DIR}/src/QuadProg++.hh
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(quadruped)

ament_package()
