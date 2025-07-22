project (common)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_NANO_LOG)
  message("ENABLE_NANO_LOG")
else()
  message("UNENABLE_NANO_LOG")
endif()


set(MDC_JSON_DEBUG_FLAG False)

if (PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_JSON_DEBUG)
  message("cyber json debug")
else()
  if (MDC_JSON_DEBUG_FLAG)
    add_definitions(-DENABLE_JSON_DEBUG)
    message("use huawei ap json debug")
  endif()
endif()

set(MDC_PROTO_DEBUG_FLAG True)
if (PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_PROTO_DEBUG)
  message("cyber proto debug")
else()
  if (MDC_PROTO_DEBUG_FLAG)
    add_definitions(-DENABLE_PROTO_DEBUG)
    message("use huawei ap proto debug")
  endif()
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
  message("ENABLE_IFLYTEK_LOG_SRC_COMMON_CMAKE")
else()
  message("UNENABLE_IFLYTEK_LOG_SRC_COMMON_CMAKE")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ilqr_lib
  rapidjson
  jlt_lib
  idm_lib
  # autoplt
  pure_pursuit_lib
  lat_lon_vehicle_motion_simulator_lib
  mjson
  planning_proto
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})