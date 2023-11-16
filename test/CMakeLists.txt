project (unit_test)
cmake_minimum_required (VERSION 3.0)

add_definitions(-DX86)
add_definitions(-DCYBER_ENV)

include_directories(
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR}
)


file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/tasks/jerk_limited_trajectory_test.cc
)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/main_test.cc
  ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
  common
  modules
  modules_scenario
  modules_tasks
  modules_common
  modules_context
  fastrtps
  cyber
  autoplt
  gtest)
