cmake_minimum_required(VERSION 3.10.2)
project(unitree_motor_sdk_m8010_6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

include_directories(include)
link_directories(lib)

#example
add_executable(motorctrl example/main.cpp)
target_link_libraries(motorctrl libUnitreeMotorSDK_M80106_Arm64.so)

#example
add_executable(stop example/stop.cpp)
target_link_libraries(stop libUnitreeMotorSDK_M80106_Arm64.so)
