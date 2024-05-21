cmake_minimum_required(VERSION 3.10)
project(gazebo_examples)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Create executable
add_executable(publisher src/publisher.cpp)
add_executable(subscriber src/subscriber.cpp)

# Link dependencies
ament_target_dependencies(publisher
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros)

ament_target_dependencies(subscriber
  rclcpp
  gazebo_msgs
  std_msgs
  geometry_msgs
  gazebo_ros)

# Install targets
install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
