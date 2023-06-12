project (unit_test)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_FLAGS "-O0 -DX86 -g")

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/**.cc
)

add_executable(${PROJECT_NAME} EXCLUDE_FROM_ALL
  ${CMAKE_CURRENT_LIST_DIR}/main_test.cc
  ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
  common
  modules
  modules_scenario
  modules_tasks
  modules_common
  modules_context
  fastrtps
  gtest)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    CONFIG_PATH="/asw/planning/res/conf")
