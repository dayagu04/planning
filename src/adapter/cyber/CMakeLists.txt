project (planning_component)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/planning_component.cc
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  proto_interface
  common
  modules
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_link_libraries(${PROJECT_NAME} PUBLIC
    NanoLog
  )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    CONFIG_PATH="/asw/planning/res/conf")