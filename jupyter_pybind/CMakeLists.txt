project(jupyter_pybind)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)
set(PYBIND11_PYTHON_VERSION 3.7)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/notebooks/scripts/")
set(SH_COMMAND "python proto_gen.py")
set(CMAKE_CXX_FLAGS "-DX86")

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
  modules_context
  modules_tasks)

pybind11_add_module(longitudinal_motion_planning_py MODULE src/longitudinal_motion_planning_pybind.cc)
target_link_libraries(longitudinal_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(slot_management_py MODULE src/slot_management_pybind.cc)
target_link_libraries(slot_management_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(diag_slot_planning_py MODULE src/diag_slot_planning_pybind.cc)
target_link_libraries(diag_slot_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(uss_obstacle_avoidance_py MODULE src/uss_obstacle_avoidance_pybind.cc)
target_link_libraries(uss_obstacle_avoidance_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(dubins_lib_py MODULE src/dubins_lib_pybind.cc)
target_link_libraries(dubins_lib_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(collision_detection_py MODULE src/collision_detection_pybind.cc)
target_link_libraries(collision_detection_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)
