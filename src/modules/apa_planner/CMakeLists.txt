project (modules_apa_planner)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
)

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/src/lateral_path_optimizer
  ${CMAKE_CURRENT_LIST_DIR}/src/apa_world
  ${CMAKE_CURRENT_LIST_DIR}/src/perpendicular_planner
  ${CMAKE_CURRENT_LIST_DIR}/src/parallel_planner)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  framework
  common
  modules_context
  modules_common
  geometry_lib
)

target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
