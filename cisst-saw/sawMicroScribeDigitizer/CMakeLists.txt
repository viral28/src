#
#
# Author(s):  Min Yang Jung
# Created on: 2011-10-06
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# To find FindMicroScribe.cmake
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/code)

find_package(MicroScribe REQUIRED)

if (MICROSCRIBE_FOUND)
  include_directories (${CMAKE_CURRENT_BINARY_DIR} ${MICROSCRIBE_INCLUDE_DIR})
  add_subdirectory (code)
  add_subdirectory (examples)
endif (MICROSCRIBE_FOUND)
