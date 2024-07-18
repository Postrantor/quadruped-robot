#
# This file is part of qpOASES.
#
# qpOASES -- An Implementation of the Online Active Set Strategy. Copyright (C)
# 2007-2017 by Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.
# All rights reserved.
#
# qpOASES is free software; you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation; either version 2.1 of the License, or (at your option)
# any later version.
#
# qpOASES is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License along
# with qpOASES; if not, write to the Free Software Foundation, Inc., 51 Franklin
# Street, Fifth Floor, Boston, MA  02110-1301  USA
#

#
# Filename:  CMakeLists.txt Author:    Hans Joachim Ferreau (thanks to Milan
# Vukov) Version:   3.2 Date:      2007-2017
#

cmake_minimum_required(VERSION 3.10)
project(qpOASES CXX)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(PACKAGE_NAME "qpOASES")
set(PACKAGE_VERSION "3.2.0")
set(PACKAGE_SO_VERSION "3.2")
set(PACKAGE_DESCRIPTION "An implementation of the online active set strategy")
set(PACKAGE_AUTHOR
    "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
set(PACKAGE_MAINTAINER
    "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
set(PACKAGE_URL "https://projects.coin-or.org/qpOASES")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

if(NOT CMAKE_VERBOSE_MAKEFILE)
  set(CMAKE_VERBOSE_MAKEFILE OFF)
endif(NOT CMAKE_VERBOSE_MAKEFILE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE
        STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)
option(QPOASES_BUILD_EXAMPLES "Build examples." ON)
option(
  QPOASES_AVOID_LA_NAMING_CONFLICTS
  "If ON, avoid to re-defined symbols that conflict with Blas/Lapack provided functions."
  OFF)

if(BUILD_SHARED_LIBS AND WIN32)
  message(
    FATAL_ERROR
      "Compiling qpOASES as a shared library in Windows is not supported.")
endif()

# ##############################################################################
# ################### compiler flags ########################
# ##############################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
if(QPOASES_AVOID_LA_NAMING_CONFLICTS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__AVOID_LA_NAMING_CONFLICTS__")
endif()

if(UNIX)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
elseif(WINDOWS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

# ##############################################################################
# ####################### rpath #############################
# ##############################################################################
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already (but later on when
# installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH
    "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")

# add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
     "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH
      "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")
endif("${isSystemDir}" STREQUAL "-1")

# ##############################################################################
# ################### build and install #####################
# ##############################################################################
include_directories(${PROJECT_SOURCE_DIR}/include)

# compile qpOASES libraries
file(GLOB SRC src/*.cpp)

# library
add_library(${PROJECT_NAME} SHARED ${SRC})
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib)
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PACKAGE_SO_VERSION})

install(
  DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

# ##############################################################################
# ######################## examples #########################
# ##############################################################################

# if(QPOASES_BUILD_EXAMPLES)
#   # compile qpOASES examples
#   set(EXAMPLE_NAMES
#       example1
#       example1a
#       example1b
#       example2
#       example3
#       example3b
#       example4
#       example5
#       exampleLP
#       qrecipe
#       qrecipeSchur)

#   foreach(ELEMENT ${EXAMPLE_NAMES})
#     add_executable(${ELEMENT} examples/${ELEMENT}.cpp)
#     target_link_libraries(${ELEMENT} qpOASES)
#   endforeach(ELEMENT ${EXAMPLE_NAMES})
# endif(QPOASES_BUILD_EXAMPLES)

#
# end of file
#
