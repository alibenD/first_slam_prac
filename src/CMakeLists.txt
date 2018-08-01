#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-08-01 16:04:18
# @last_modified_date: 2018-08-01 16:05:24
# @description: TODO
#---****************************************************************---

# Cmake version required

ADD_LIBRARY(myslam SHARED
  frame.cc
  mappoint.cc
  map.cc
  camera.cc
  config.cc)
# Set Dependence PATH
  # Example: set ( PACKAGE_DIR PATH )

# Find *Config.cmake
  # Example: find_package(OpenCV REQUIRED)

# EXECUTABLE
  # Example: add_executable( EXEC_NAME SRC_FILE_NAME_LIST )

# TARGET LINK
  # Example: target_link_libraries( EXEC_NAME LIBPATH ) ...LIB_PATH e.g. ${OPENCV_LIBS}
  TARGET_LINK_LIBRARIES(myslam ${THIRD_PARTY_LIBS})


