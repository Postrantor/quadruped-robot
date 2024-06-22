cmake_minimum_required(VERSION 3.5)
project(qr_gazebo)

# 查找需要的 ROS2 包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(gazebo REQUIRED)

# 定义依赖项
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  unitree_legged_msgs
  gazebo
)

# 包含目录
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${unitree_legged_msgs_INCLUDE_DIRS}
)

# 定义共享库和可执行文件的源文件
set(PLUGINS
  plugin/foot_contact_plugin.cc
  plugin/draw_force_plugin.cc
  plugin/draw_lines_plugin.cc
)

# 创建共享库
foreach(PLUGIN ${PLUGINS})
  get_filename_component(LIB_NAME ${PLUGIN} NAME_WE)
  add_library(${LIB_NAME} SHARED ${PLUGIN})
  ament_target_dependencies(${LIB_NAME} ${DEPENDENCIES})
  target_link_libraries(${LIB_NAME} ${GAZEBO_LIBRARIES})
endforeach()

set(EXECUTABLES
  node/spawn_model.cpp
)

# 创建可执行文件
foreach(EXEC ${EXECUTABLES})
  get_filename_component(EXEC_NAME ${EXEC} NAME_WE)
  add_executable(${EXEC_NAME} ${EXEC})
  ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})
endforeach()

# 创建库
file(GLOB_RECURSE sources "src/*.cpp")
add_library(qr_gazebo SHARED ${sources})
ament_target_dependencies(qr_gazebo ${DEPENDENCIES})

# 安装
install(TARGETS
  qr_gazebo
  unitreeFootContactPlugin
  unitreeDrawForcePlugin
  drawLinesPlugin
  spawn_model
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
