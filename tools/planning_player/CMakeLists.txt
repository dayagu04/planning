project (pp)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-O0 -DX86 -g")

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  struct_msgs
  proto_msgs
  sensor_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../../interface/src
  ${CMAKE_CURRENT_LIST_DIR}/../../interface/type_convert
)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  modules
  cyber
  fastrtps #?
  autoplt
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
