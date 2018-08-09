#!/bin/bash

#---Automatically Generated from template 'bash' wrote by @aliben---
# @Copyright (C) 2018 All rights reserved.
# @file: FindCSparse.cmake
# @author: aliben.develop@gmail.com
# @created_date: 2018-08-01 15:09:30
# @last_modified_date: NO_LAST_MODIFIED_DATE
# @brief: TODO
# @details: TODO
#---***********************************************---


#---Variables

#---CMake Command

FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  /usr/include
  /usr/include/ufsparse
  /usr/include/suitesparse
  /usr/local/include
  /usr/local/include/ufsparse
  /opt/local/include
  /opt/local/include/ufsparse
  /sw/include
  /sw/include/ufsparse
  /Users/aliben/dev/env/include/EXTERNAL
  /Users/aliben/dev/env/include
  /Users/aliben/dev/env/homebrew/include
  )

FIND_LIBRARY(CSPARSE_LIBARAY NAMES cxsparse
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /Users/aliben/dev/env/lib
  /Users/aliben/dev/env/homebrew/lib
  /sw/lib
  )
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CSPARSE DEFAULT_MSG
  CSPARSE_INCLUDE_DIR CSPARSE_LIBRARY)
