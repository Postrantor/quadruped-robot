cmake_minimum_required(VERSION 3.5)
project(qr_gazebo)

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(controller_interface REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(unitree_msgs REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  gazebo
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
  unitree_msgs
  controller_manager_msgs
  controller_interface
)

# include
include_directories(
  "include"
  ${GAZEBO_INCLUDE_DIRS})

# plugin_source
set(PLUGINS
  plugin/foot_contact_plugin.cpp
  plugin/draw_force_plugin.cpp
  plugin/draw_lines_plugin.cpp
)

# library
add_library(qr_gazebo SHARED
  "src/gazebo_model_spawn.cpp"
)
ament_target_dependencies(qr_gazebo ${DEPENDENCIES})

# plugins
add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cpp)
ament_target_dependencies(unitreeFootContactPlugin ${DEPENDENCIES})
add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cpp)
ament_target_dependencies(unitreeDrawForcePlugin ${DEPENDENCIES})
add_library(drawLinesPlugin SHARED plugin/draw_lines_plugin.cpp)
ament_target_dependencies(drawLinesPlugin ${DEPENDENCIES})

# executable
add_executable(spawn_model
  "src/spawn_model.cpp"
)
target_link_libraries(spawn_model qr_gazebo)
ament_target_dependencies(spawn_model ${DEPENDENCIES})

# install
install(
  TARGETS
    spawn_model
    qr_gazebo
    unitreeFootContactPlugin
    unitreeDrawForcePlugin
    drawLinesPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
