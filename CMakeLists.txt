# ----------------------------------------------------------------------------
#  Sources CMake file for C++ api Library
#
#    From the api_cpp/examples directory, invoke:
#      $ mkdir build
#      $ cd build/
#      $ cmake ..
#      $ make
#
#  Copyright © 2018 Kinova Robotics
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE release)
endif()

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/../bin")
MESSAGE("Setting EXECUTABLE_OUTPUT_PATH: '${EXECUTABLE_OUTPUT_PATH}'")

if(NOT KORTEX_SUB_DIR)
  set(KORTEX_SUB_DIR "")
else()
  set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
endif()

add_subdirectory(src)
