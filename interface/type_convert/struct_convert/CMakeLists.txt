project(struct_convert)

add_library(struct_convert INTERFACE)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  struct_msgs
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_include_directories(struct_convert INTERFACE 
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/..
${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

target_link_libraries(struct_convert INTERFACE c_interface)