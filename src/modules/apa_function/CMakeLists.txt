project (modules_apa_function)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
)

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/src/apa_world/lateral_path_optimizer
  ${CMAKE_CURRENT_LIST_DIR}/src/apa_world
  ${CMAKE_CURRENT_LIST_DIR}/src/perpendicular_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/perpendicular_planner/perpendicular_park_in_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/perpendicular_planner/perpendicular_park_heading_in_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/perpendicular_planner/perpendicular_park_out_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/parallel_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/hybrid_astar_planner)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  framework
  modules_context
  modules_common
  geometry_lib
  modules_base_function
  planning_proto
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
)

target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
