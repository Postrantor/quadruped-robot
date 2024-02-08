cmake_minimum_required(VERSION 3.5)
project(unitree_legged_sdk)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 17)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)

find_package(ament_cmake REQUIRED)

# set
set(ARCH arm64)
include_directories(include)
link_directories(lib/cpp/${ARCH})

option(PYTHON_BUILD "build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)
endif()

# exec
add_executable(example_position src/example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity src/example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque src/example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk src/example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_joystick src/example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})

# install
install(
  TARGETS
    example_position
    example_velocity
    example_torque
    example_walk
    example_joystick
  DESTINATION bin
)

install(
  DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION .
)

# export
ament_export_include_directories(include)
ament_export_libraries(unitree_legged_sdk)
# ament_export_dependencies(unitree_legged_sdk)
ament_package()
