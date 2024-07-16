cmake_minimum_required(VERSION 3.8)
project(mit_amd)

find_package(ament_cmake REQUIRED)

include_directories(include)

file(GLOB_RECURSE sources src/*.c)
add_library(${PROJECT_NAME} SHARED ${sources})

target_link_libraries(${PROJECT_NAME}
  pthread
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
