project(ifly_glog)

file(GLOB ifly_glog_file
    "*.cc"
    "cyber/logger/*.cc"
)

set(MDC_AP_LOG_FLAG True)

if (PLATFORM STREQUAL "X86")
  add_definitions(-DIFLY_GLOG_PATH)
  message("cyber config")
else()
  if (MDC_AP_LOG_FLAG)
    add_definitions(-DENABLE_MDC_AP_LOG)
    message("use huawei ap log")
  else()
    message("use google log")
  endif()
endif()

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