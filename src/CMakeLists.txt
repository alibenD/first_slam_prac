#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-08-01 16:04:18
# @last_modified_date: 2018-09-10 14:36:39
# @description: TODO
#---****************************************************************---

# Cmake version required
FILE(GLOB TYPES_HEADERS
  ../include/myslam/*/*.h
  ../include/myslam/*/*.hh
  ../include/myslam/*/*.hpp
  ../include/myslam/*.h
  ../include/myslam/*.hh
  ../include/myslam/*.hpp
  )

FILE(GLOB SOURCES *.c *.cc *.cpp)


#ADD_LIBRARY(myslam SHARED
#  frame.cc
#  mappoint.cc
#  map.cc
#  camera.cc
#  config.cc
#  visual_odometry.cc
#  g2o_types.cc)
ADD_LIBRARY(myslam SHARED ${TYPES_HEADERS} ${SOURCES})

TARGET_LINK_LIBRARIES(myslam ${THIRD_PARTY_LIBS})
