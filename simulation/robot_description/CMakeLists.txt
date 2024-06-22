cmake_minimum_required(VERSION 3.5)
project(robot_description)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    xacro
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
