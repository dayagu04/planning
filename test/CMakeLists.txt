project (unit_test)
cmake_minimum_required (VERSION 3.0)

add_compile_options(-std=c++17 -Wall -g -O0 -fPIC -fpermissive -DCYBER_ENV -DLINUX)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/scenario/scenario_state_machine_test.cc)

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${CMAKE_CURRENT_LIST_DIR}/main_test.cc)
target_link_libraries(${PROJECT_NAME}
  planning_component
  fastrtps
  gtest)

