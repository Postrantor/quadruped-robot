# This software may be modified and distributed under the terms of the MIT
# license.  See the LICENSE file for details.
cmake_minimum_required(VERSION 3.8)
project(quadprog CXX)

find_package(ament_cmake REQUIRED)

include_directories(include)

file(GLOB sources src/*.cc)
file(GLOB headers include/*.hh)
set(library_sources ${sources})
list(REMOVE_ITEM library_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc)
add_library(quadprog SHARED ${library_sources} ${headers})
set_property(TARGET quadprog PROPERTY POSITION_INDEPENDENT_CODE 1)
set_property(TARGET quadprog PROPERTY PUBLIC_HEADER ${headers})

# add_executable(main main.cc ${headers}) target_link_libraries(main quadprog)

install(
  TARGETS quadprog
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)
ament_export_libraries(quadprog)

ament_package()
