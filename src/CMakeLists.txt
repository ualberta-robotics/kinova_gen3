cmake_minimum_required(VERSION 3.10)

project(Getting_started)
MESSAGE("---------------------------------------------------------------------------------")
MESSAGE("Project '${PROJECT_NAME}' args: ${CMAKE_ARGS}")
MESSAGE("Project building for: '${CMAKE_SYSTEM_NAME}' in '${CMAKE_BUILD_TYPE}' mode")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE release)
endif()

if(NOT KORTEX_SUB_DIR)
  set(KORTEX_SUB_DIR "")
else()
  set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
endif()

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/../kortex_api/${KORTEX_SUB_DIR}")
MESSAGE("create KORTEX_DIR: '${KORTEX_DIR}'")

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_AUTOMOC               ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)
set(BUILD_SHARED_LIBS           OFF)

set(HEADERS_DIR "${PROJECT_SOURCE_DIR}/../include/")
set(SOURCES ${HEADERS_DIR}joystickBox.cc ${HEADERS_DIR}joystickBox.hh)

set(TARGET_EXE_NAME "SpaceMouseExample")
add_executable(${TARGET_EXE_NAME} simple.cpp)

target_compile_options(${TARGET_EXE_NAME} PRIVATE -Wall)
target_compile_options(${TARGET_EXE_NAME} PRIVATE -Wno-reorder)
target_compile_features(${TARGET_EXE_NAME} PRIVATE cxx_std_11)
target_compile_definitions(${TARGET_EXE_NAME} PRIVATE -D_OS_UNIX)

target_link_libraries(${TARGET_EXE_NAME} PRIVATE pthread spnav)
target_include_directories(${TARGET_EXE_NAME} PRIVATE ${HEADERS_DIR})

target_include_directories(${TARGET_EXE_NAME} PRIVATE ${KORTEX_DIR}include/client_stubs)


