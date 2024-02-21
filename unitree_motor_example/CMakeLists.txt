cmake_minimum_required(VERSION 3.16)
project(unitree_motor_example LANGUAGES CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  unitree_msgs
  unitree_sdk
)

set(MOTOR_SDK
  -pthread
  lcm
  ${MOTORLIB}
  ${LEGGEDLIB}
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_library(LEGGEDLIB unitree_legged_sdk REQUIRED)
find_library(MOTORLIB UnitreeMotorSDK_M80106_Arm64 REQUIRED)

## COMPILE
add_library(
  unitree_motor_example SHARED
  hardware/unitree_motor_system.cpp
)
# target_compile_features(unitree_motor_example PUBLIC cxx_std_17)
target_include_directories(unitree_motor_example PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/unitree_motor_example>
)
target_link_libraries(unitree_motor_example PUBLIC ${MOTOR_SDK})
ament_target_dependencies(
  unitree_motor_example PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "UNITREE_MOTOR_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface unitree_motor_example.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/unitree_motor_example
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/unitree_motor_example
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/unitree_motor_example
)
install(TARGETS unitree_motor_example
  EXPORT export_unitree_motor_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
# endif()

## EXPORTS
ament_export_targets(export_unitree_motor_example HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
