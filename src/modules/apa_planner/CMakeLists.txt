project (modules_apa_planner)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
  ${CMAKE_CURRENT_LIST_DIR}/**.cpp
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  framework
  common
  modules_context
  modules_common
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
