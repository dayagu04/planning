project (modules)
cmake_minimum_required(VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

execute_process (
  COMMAND git rev-parse --short=8 --vertify HEAD
  OUTPUT_VARIABLE SOURCE_COMMIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
)
message (STATUS "SOURCE_COMMIT(git)=" ${SOURCE_COMMIT})

configure_file (
  "${CMAKE_CURRENT_LIST_DIR}/version.h.in"
  "${CMAKE_CURRENT_LIST_DIR}/version.h"
)

add_subdirectory(context EXCLUDE_FROM_ALL)
add_subdirectory(apa_planner EXCLUDE_FROM_ALL)
add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(scenario EXCLUDE_FROM_ALL)
add_subdirectory(scc_function EXCLUDE_FROM_ALL)
add_subdirectory(tasks EXCLUDE_FROM_ALL)

target_link_libraries(${PROJECT_NAME} PUBLIC
  framework
  modules_scenario
  modules_apa_planner
  hfsm
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
