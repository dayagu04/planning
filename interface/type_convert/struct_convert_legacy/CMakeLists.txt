project(struct_convert_legacy)

add_library(struct_convert_legacy INTERFACE)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  struct_msgs_legacy_v2_4_5
  struct_msgs_legacy_v2_4_6
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_include_directories(struct_convert_legacy INTERFACE 
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/..
${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

target_link_libraries(struct_convert_legacy INTERFACE legacy_c_interface)