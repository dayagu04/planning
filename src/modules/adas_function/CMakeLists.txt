project (modules_adas_function)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/ihc_function/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/tsr_function/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/lkas_function/*.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  modules_common
  modules_tasks
  framework
  modules_base_task_pipeline
  modules_base_function
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
