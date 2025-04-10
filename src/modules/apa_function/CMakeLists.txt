project (modules_apa_function)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/apa_common
  ${CMAKE_CURRENT_LIST_DIR}/collision_detection
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/parallel
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/perpendicular
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/perpendicular/head_in
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/perpendicular/head_out
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/perpendicular/tail_in
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/perpendicular/tail_out
  ${CMAKE_CURRENT_LIST_DIR}/parking_scenario/narrow_space
  ${CMAKE_CURRENT_LIST_DIR}/parking_task
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/perpendicular
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/perpendicular/head_in
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/perpendicular/head_out
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/perpendicular/tail_in
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/perpendicular/tail_out
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/geometry_path_generators/parallel
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/optimizers
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/optimizers/lateral_path_optimizer
  ${CMAKE_CURRENT_LIST_DIR}/util
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders/parking_stop_decider
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders/virtual_wall_decider
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders/narrow_space_decider
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders/speed_limit_decider
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/deciders/traj_stitcher
  ${CMAKE_CURRENT_LIST_DIR}/parking_task/optimizers/sv_dp_optimizer
  )

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  framework
  modules_context
  modules_common
  geometry_lib
  modules_base_function
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
)

target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})