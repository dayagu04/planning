project (planning_component)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  -Wl,-whole-archive
  modules
  -Wl,-no-whole-archive
  autoplt
  osqpstatic
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT PLATFORM STREQUAL "X86")
  target_link_libraries(${PROJECT_NAME} PUBLIC
    NanoLog
  )
endif()
