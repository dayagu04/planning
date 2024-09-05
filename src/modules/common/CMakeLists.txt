project (modules_common)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  modules_context
  c_interface
  legacy_c_interface
  planning_proto
  mjson
  framework
  advanced_ctrl_lib
  osqp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
