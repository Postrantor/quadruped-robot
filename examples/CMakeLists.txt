cmake_minimum_required(VERSION 3.5)
project(robot_sim)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Wno-unused-variable)
endif()

# find_package
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadruped REQUIRED)
find_package(unitree_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# find_package(unitree_legged_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  backward_ros
  quadruped
  unitree_msgs
  gazebo_msgs
  std_srvs
  sensor_msgs
  nav_msgs
)

include_directories(include)

add_executable(example_keyboard
  simulation/example_keyboard.cpp
)
ament_target_dependencies(example_keyboard ${DEPENDENCIES})

# control a entity robot in gazebo
add_executable(example_gazebo
  simulation/example_gazebo.cpp)
target_link_libraries(example_gazebo
  Eigen3::Eigen
)
ament_target_dependencies(example_gazebo ${DEPENDENCIES})

# control a joint in gazebo
add_executable(example_joint
  simulation/example_joint.cpp
)
ament_target_dependencies(example_joint ${DEPENDENCIES})

install(
  TARGETS
    example_joint
    example_gazebo
    example_keyboard
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# try set position in gazebo
install(
  PROGRAMS
    simulation/example_set_entity.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
