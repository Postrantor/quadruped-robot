cmake_minimum_required(VERSION 3.10)
project(gazebo_examples)

# find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

# include directory
# include_directories(
#   ${CMAKE_SOURCE_DIR}/include
# )

# create executable
add_executable(publisher src/publisher.cpp)
add_executable(subscriber src/subscriber.cpp)

add_library(gazebo_ros_diff_drive SHARED
  src/gazebo_ros_diff_drive.cpp
)
target_include_directories(gazebo_ros_diff_drive PUBLIC include)
ament_target_dependencies(gazebo_ros_diff_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_diff_drive)
# if(ENABLE_PROFILER)
#   target_include_directories(gazebo_ros_diff_drive PUBLIC ${ignition-common3_INCLUDE_DIRS})
#   target_link_libraries(gazebo_ros_diff_drive ${ignition-common3_LIBRARIES})
# endif()

# link dependencies
ament_target_dependencies(publisher
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
)
ament_target_dependencies(subscriber
  rclcpp
  gazebo_msgs
  std_msgs
  geometry_msgs
  gazebo_ros
)

# install targets
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    publisher
    subscriber
    gazebo_ros_diff_drive
  # DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# ament package
ament_package()
