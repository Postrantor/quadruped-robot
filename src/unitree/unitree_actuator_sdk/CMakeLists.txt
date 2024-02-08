cmake_minimum_required(VERSION 3.1.0)
project(unitree_actuator_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include_directories(include)
link_directories(lib)

set(MOTOR_SDK libUnitreeMotorSDK_Arm64.so)

#example
add_executable(go_m8010_6 src/example_go_m8010_6.cpp)
target_link_libraries(go_m8010_6 ${MOTOR_SDK})

add_executable(change_id src/change_id.cpp)
target_link_libraries(change_id ${MOTOR_SDK})

# 
set(LIBRARY_OUTPUT_PATH "../lib")
add_subdirectory(thirdparty/pybind11)
pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)
target_link_libraries(unitree_actuator_sdk PRIVATE ${MOTOR_SDK})
set_target_properties(unitree_actuator_sdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")

# 
set(EXTRA_LIBS2 libUnitree_motor_SDK_ARM64.so)
add_executable(check_c src/check.c)
target_link_libraries(check_c ${EXTRA_LIBS2})
add_executable(check_cpp src/check.cpp)
target_link_libraries(check_cpp ${EXTRA_LIBS2})

# 
install(
    TARGETS 
        go_m8010_6
        change_id
        check_c
        check_cpp
    RUNTIME DESTINATION bin
    # LIBRARY DESTINATION lib
    # ARCHIVE DESTINATION lib
    # INCLUDES include
)

install(
    DIRECTORY lib
    DESTINATION lib
)

# ament_package()
