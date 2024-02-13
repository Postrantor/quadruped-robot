cmake_minimum_required(VERSION 3.5)
project(model_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rrbot/urdf rrbot/rviz
  DESTINATION share/${PROJECT_NAME}/rrbot
)

install(
  DIRECTORY unitree/urdf unitree/rviz
  DESTINATION share/${PROJECT_NAME}/unitree
)

install(
  DIRECTORY r6bot/meshes r6bot/srdf r6bot/urdf r6bot/rviz
  DESTINATION share/${PROJECT_NAME}/r6bot
)

ament_package()
