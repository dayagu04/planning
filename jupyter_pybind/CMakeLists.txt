project(jupyter_pybind)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)
set(PYBIND11_PYTHON_VERSION 3.7)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "-DX86")
set(WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/notebooks_apa/scripts/")
set(SH_COMMAND "python proto_gen.py")

execute_process(
  COMMAND sh -c "${SH_COMMAND}"
  RESULT_VARIABLE SH_RESULT
  OUTPUT_VARIABLE SH_OUTPUT
  WORKING_DIRECTORY ${WORKING_DIRECTORY}
)

set(WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/notebooks_scc/scripts/")
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

pybind11_add_module(perpendicular_planning_py MODULE src/perpendicular_planning_pybind.cc)
target_link_libraries(perpendicular_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(slant_planning_py MODULE src/slant_planning_pybind.cc)
target_link_libraries(slant_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(vertical_planning_py MODULE src/vertical_planning_pybind.cc)
target_link_libraries(vertical_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(parallel_planning_py MODULE src/parallel_planning_pybind.cc)
target_link_libraries(parallel_planning_py PRIVATE
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

pybind11_add_module(apa_simulation_py MODULE src/apa_simulation_pybind.cc)
target_link_libraries(apa_simulation_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(geometry_math_validation_py MODULE src/geometry_math_validation_pybind.cc)
target_link_libraries(geometry_math_validation_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(lateral_path_optimizer_py MODULE src/lateral_path_optimizer_pybind.cc)
target_link_libraries(lateral_path_optimizer_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(optimizer_apa_simulation_py MODULE src/optimizer_apa_simulation_pybind.cc)
  target_link_libraries(optimizer_apa_simulation_py PRIVATE
    autoplt
    modules
    modules_common
    modules_context
    modules_tasks)

pybind11_add_module(jerk_limited_trajectory_py MODULE src/jerk_limited_trajectory_pybind.cc)
target_link_libraries(jerk_limited_trajectory_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(gap_selector_py MODULE src/gap_selector_pybind.cc)
target_link_libraries(gap_selector_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)
