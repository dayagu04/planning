project(planning)
cmake_minimum_required(VERSION 3.0)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(ad_common)

if (PP_ENABLE)
  add_subdirectory(tools/planning_player)
endif()

if (PYBIND_TOOL_ENABLE)
  add_subdirectory(jupyter_pybind)
endif()