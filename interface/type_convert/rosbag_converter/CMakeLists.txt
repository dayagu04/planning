project (rosbag_converter)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-O2 -g")

add_subdirectory(../private_proto private_proto.out)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  sensor_interface
)

if(CHECK_MODULE)
  function(check_for_warnings FILE)
      file(READ ${FILE} FILE_CONTENT)
      string(FIND "${FILE_CONTENT}" "// warning :" WARNING_FOUND)
      if(WARNING_FOUND GREATER -1)
          message(SEND_ERROR "File ${FILE} contains a warning comment.")
      endif()
  endfunction()

  file(GLOB_RECURSE SRC_FILES "../ros_convert/${CHECK_MODULE}")
  foreach(SRC_FILE ${SRC_FILES})
      check_for_warnings(${SRC_FILE})
  endforeach()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../install/bin)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  private_proto
  ros_convert
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})