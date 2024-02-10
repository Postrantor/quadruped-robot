cmake_minimum_required(VERSION 3.5.0)
project(unitree_actuator_sdk)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include_directories(include)
link_directories(lib)

#example
set(MOTOR_SDK libUnitreeMotorSDK_M80106_Arm64.so)
add_executable(go_m8010_6 src/example_go_m8010_6.cpp)
target_link_libraries(go_m8010_6 ${MOTOR_SDK})

add_executable(change_id src/change_id.cpp)
target_link_libraries(change_id ${MOTOR_SDK})

# 
# set(EXTRA_LIBS2 libUnitree_motor_SDK_ARM64.so)
# add_executable(check_c src/check.c)
# target_link_libraries(check_c ${EXTRA_LIBS2})
# add_executable(check_cpp src/check.cpp)
# target_link_libraries(check_cpp ${EXTRA_LIBS2})

# 
install(
  TARGETS 
    go_m8010_6
    # change_id
    # check_c
    # check_cpp
  RUNTIME DESTINATION bin
  # LIBRARY DESTINATION lib
  # ARCHIVE DESTINATION lib
  # INCLUDES include
)

install(
  DIRECTORY lib DESTINATION .
  DIRECTORY include DESTINATION .
)

ament_export_include_directories(include)
ament_export_libraries(libUnitreeMotorSDK_M80106_Arm64.so)
ament_package()
