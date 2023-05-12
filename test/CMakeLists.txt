project (unit_test)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/scenario/scenario_state_machine_test.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/tasks/lateral_motion_planning_cost_test.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/tasks/longitudinal_motion_planning_cost_test.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/tasks/lateral_motion_planning_one_frame_test.cc)

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${CMAKE_CURRENT_LIST_DIR}/main_test.cc)

target_link_libraries(${PROJECT_NAME}
  planning_component
  fastrtps
  gtest)

