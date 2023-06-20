project(jupyter_pybind)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)
set(PYBIND11_PYTHON_VERSION 3.7)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/notebooks/scripts/")
set(SH_COMMAND "python proto_gen.py")

execute_process(
  COMMAND sh -c "${SH_COMMAND}"
  RESULT_VARIABLE SH_RESULT
  OUTPUT_VARIABLE SH_OUTPUT
  WORKING_DIRECTORY ${WORKING_DIRECTORY}
)

find_package(pybind11 REQUIRED PATHS ~/miniconda3/lib/python3.7/site-packages/pybind11/share/cmake/pybind11)

pybind11_add_module(lateral_motion_planning_py MODULE src/lateral_motion_planning_pybind.cc)
target_link_libraries(lateral_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_tasks)

pybind11_add_module(longitudinal_motion_planning_py MODULE src/longitudinal_motion_planning_pybind.cc)
target_link_libraries(longitudinal_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_tasks)

