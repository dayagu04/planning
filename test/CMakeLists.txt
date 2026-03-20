project (unit_test)
cmake_minimum_required (VERSION 3.0)

add_definitions(-DX86)
add_definitions(-DCYBER_ENV)

include_directories(
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR}
)

# file(GLOB_RECURSE SRC_FILES
#   ${CMAKE_CURRENT_LIST_DIR}/src/modules/planning/lateral_path_optimizer_cost_test.cc
# )
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/modules/planning/lateral_path_optimizer_cost_test.cc
)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/main_test.cc
  ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
  common
  modules
  modules_tasks
  modules_common
  modules_context
  fastrtps
  cyber
  autoplt
  gtest)

add_executable(test_cyber_talker src/modules/utils/test_cyber_talker.cc)
target_link_libraries(test_cyber_talker
  cyber
  cyber_proto
  fastrtps
  pthread
  protobuf
  gflags
  glog
  common
)

add_executable(test_cyber_listener src/modules/utils/test_cyber_listener.cc
)
target_link_libraries(test_cyber_listener
  cyber
  cyber_proto
  fastrtps
  pthread
  protobuf
  gflags
  glog
  common
)

add_executable(test_cyber_log src/modules/utils/test_cyber_log.cc
)
target_link_libraries(test_cyber_log
  cyber
  cyber_proto
  gflags
  glog
  common
  pthread
)


find_package(OpenCV REQUIRED)

add_executable(test_opencv2d
  ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/utils/test_opencv.cc
  )
target_include_directories(test_opencv2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${OpenCV_INCLUDE_DIRS}
  )
target_link_libraries(test_opencv2d PUBLIC
  ${OpenCV_LIBRARIES}
  pthread
  protobuf
  gflags
  glog
  common
  collision_detection
  )

add_executable(test_edt
  ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/utils/test_edt.cc
  )
target_include_directories(test_edt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${OpenCV_INCLUDE_DIRS}
  )
target_link_libraries(test_edt PUBLIC
  ${OpenCV_LIBRARIES}
  pthread
  protobuf
  gflags
  glog
  common
  collision_detection
  )

add_subdirectory(src/modules/openspace)


set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
          test_opencv2d
          test_cyber_talker
          test_cyber_listener
          test_cyber_log
          test_edt
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


