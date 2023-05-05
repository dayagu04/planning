cmake_minimum_required(VERSION 3.0)

project(planning)
include_directories(${CMAKE_CURRENT_LIST_DIR}/res/include/proto)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(USE_UNIT_TEST ON)

add_subdirectory(src)

# unit test
if (USE_UNIT_TEST)
  add_subdirectory(test)
endif()
