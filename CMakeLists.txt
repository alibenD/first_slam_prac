#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-08-01 15:25:28
# @last_modified_date: 2018-08-23 23:30:28
# @description: TODO
#---****************************************************************---

# Cmake version required
cmake_minimum_required(VERSION 2.8)

#project name
project(myslam)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-g -std=c++14 -march=native -O0")

SET(CMAKE_VERVOSE_MAKEFILE ON)

if (CMAKE_HOST_WIN32)
    set(WINDOWS 1)
elseif(CMAKE_HOST_APPLE)
    set(MACOS 1)
elseif(CMAKE_HOST_UNIX)
    set(LINUX 1)
endif()

if(WINDOWS)
else()
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    MESSAGE( STATUS "CMAKE_INSTALL_RPATH: " "${CMAKE_INSTALL_PREFIX}/lib" )
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()



list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)


# Set Dependence PATH
  # Example: set ( PACKAGE_DIR PATH )
  FIND_PACKAGE(Eigen3 REQUIRED)
  include_directories( ${EIGEN3_INCLUDE_DIRS} )
  MESSAGE(STATUS ${EIGEN3_INCLUDE_DIRS})
  FIND_PACKAGE(OpenCV REQUIRED)
  INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
  MESSAGE( STATUS "OpenCV Path: ${OpenCV_INCLUDE_DIRS}" )

  FIND_PACKAGE(Sophus REQUIRED)
  INCLUDE_DIRECTORIES( ${SOPHUS_INCLUDE_DIRS} )
  MESSAGE( STATUS "Sophus Path: ${SOPHUS_INCLUDE_DIRS}" )

  FIND_PACKAGE(G2O REQUIRED)
  INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIRS} )
  MESSAGE( STATUS "G2O Path: ${G2O_INCLUDE_DIRS}")

  SET( THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba)

# Find *Config.cmake
  # Example: find_package(OpenCV REQUIRED)

# EXECUTABLE
  # Example: add_executable( EXEC_NAME SRC_FILE_NAME_LIST )
  include_directories( ${PROJECT_SOURCE_DIR}/include )
  add_subdirectory(src)
  add_subdirectory(test)

# TARGET LINK
  # Example: target_link_libraries( EXEC_NAME LIBPATH ) ...LIB_PATH e.g. ${OPENCV_LIBS}
