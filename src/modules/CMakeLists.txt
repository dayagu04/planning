project (modules)
cmake_minimum_required(VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

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

target_compile_definitions(${PROJECT_NAME} PUBLIC
    CONFIG_PATH="${CONFIG_PATH}")
message("CONFIG_PATH=${CONFIG_PATH}")
