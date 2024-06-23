cmake_minimum_required(VERSION 3.16)
project(unitree_joint_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  generate_parameter_library
  pluginlib
  realtime_tools
  controller_interface
  hardware_interface
  geometry_msgs
  # nav_msgs
  rclcpp
  rclcpp_lifecycle
  rcpputils
  # tf2
  # tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 这里会生成一些 parameter 的类，所以下面include的时候要包含 build/
generate_parameter_library(unitree_joint_controller_parameters
  src/unitree_joint_controller_parameter.yaml
)

add_library(unitree_joint_controller SHARED
  src/unitree_joint_controller.cpp
  # src/odometry.cpp
  # src/speed_limiter.cpp
)
target_compile_features(unitree_joint_controller PUBLIC cxx_std_17)
target_include_directories(unitree_joint_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/unitree_joint_controller>
)
target_link_libraries(unitree_joint_controller PUBLIC unitree_joint_controller_parameters)
ament_target_dependencies(unitree_joint_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(unitree_joint_controller PRIVATE "unitree_joint_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface unitree_joint_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/unitree_joint_controller
)
install(TARGETS unitree_joint_controller unitree_joint_controller_parameters
  EXPORT export_unitree_joint_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_unitree_joint_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
