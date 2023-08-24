project(planning)
cmake_minimum_required(VERSION 3.0)

add_subdirectory(src)

if (UNIT_TEST_ENABLE)
  add_subdirectory(test)
endif()

if (PP_ENABLE)
  add_subdirectory(tools/planning_player)
endif()

if (PYBIND_TOOL_ENABLE)
  add_subdirectory(jupyter_pybind)
endif()