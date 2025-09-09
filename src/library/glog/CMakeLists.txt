project(ifly_glog)

file(GLOB ifly_glog_file
    "*.cc"
    "cyber/logger/*.cc"
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${ifly_glog_file})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

if (NOT PLATFORM STREQUAL "X86" AND MDC_AP_LOG_FLAG)
  target_link_libraries(${PROJECT_NAME} PUBLIC
    iflyauto_log
  )
  message("link mdc ap log")
else()
  message("unlink mdc ap log")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  glog
  gflags
  pthread
  )