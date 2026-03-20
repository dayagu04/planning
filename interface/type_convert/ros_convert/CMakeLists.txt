project(ros_convert)

add_library(ros_convert INTERFACE)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  struct_msgs
  struct_msgs_v2_10
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_include_directories(ros_convert INTERFACE 
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/..
)
