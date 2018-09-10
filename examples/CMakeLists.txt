#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-09-10 14:13:16
# @last_modified_date: 2018-09-10 14:17:42
# @description: TODO
#---****************************************************************---

# Cmake version required

FILE(GLOB SOURCES *.c *.cc *.cpp)

FOREACH(example ${SOURCES})
  GET_FILENAME_COMPONENT(example_target "${example}" NAME_WE)
  ADD_EXECUTABLE(${example_target} ${example})
  TARGET_LINK_LIBRARIES(${example_target} myslam)
ENDFOREACH()
