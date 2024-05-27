project(planning)
cmake_minimum_required(VERSION 3.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DDEBUG_PRINT_ENABLE)
add_subdirectory(src)
add_subdirectory(ad_common)

if (UNIT_TEST_ENABLE)
  add_subdirectory(test)
endif()

if (PP_ENABLE)
  add_subdirectory(tools/planning_player)
endif()

if (PYBIND_TOOL_ENABLE)
  add_subdirectory(jupyter_pybind)
endif()