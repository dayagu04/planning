project (common)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_NANO_LOG)
  message("ENABLE_NANO_LOG")
else()
  message("UNENABLE_NANO_LOG")
endif()

# t26,s811
if (ADAPTER STREQUAL "cyber")
  add_definitions(-DIFLY_LOG_PATH)
  message("cyber config")
endif()

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_IFLYTEK_LOG)
  target_link_libraries(${PROJECT_NAME} PUBLIC
    iflyauto_log
  )
  message("ENABLE_IFLYTEK_LOG")
else()
  message("UNENABLE_IFLYTEK_LOG")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  gflags
  glog
  ilqr_lib
  rapidjson
  jlt_lib
  # autoplt
  mjson
  planning_proto
  pthread
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})