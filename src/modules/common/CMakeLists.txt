project (modules_common)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  modules_context
  planning_proto
  mjson
  framework
  advanced_ctrl_lib
  osqp
  iflyauto_log
)

if(PP_ENABLE OR PYBIND_TOOL_ENABLE)
  target_link_libraries(${PROJECT_NAME} PUBLIC
    c_interface
    legacy_c_interface
  )
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC
    algo_interface
  )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
