project(ifly_glog)

file(GLOB ifly_glog_file
    "*.cc"
    "cyber/logger/*.cc"
)

if (ADAPTER STREQUAL "cyber")
  add_definitions(-DIFLY_LOG_PATH)
  message("cyber config")
endif()

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${ifly_glog_file})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

target_link_libraries(${PROJECT_NAME} PUBLIC
  glog
  gflags
  pthread
  )