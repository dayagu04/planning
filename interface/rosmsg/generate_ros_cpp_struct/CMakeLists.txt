cmake_minimum_required(VERSION 2.8.3)
project(generate_ros_cpp_struct)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  can_msgs
  tf2_msgs
  sensor_msgs
  recorder_msgs
  sensor_interface
  proto_msgs
)

set(CMAKE_TOOLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cmake_tools)
if(EXISTS ${CMAKE_TOOLS_DIR})
  include(${CMAKE_TOOLS_DIR}/generate_ros_cpp_struct_convert/generate.cmake)
  generate_ros_cpp_struct_convert(ROS_PACKAGE_NAMES
    can_msgs
    sensor_msgs
    tf2_msgs
    recorder_msgs
    sensor_interface
    proto_msgs
  )
endif()

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs can_msgs sensor_msgs
  tf2_msgs recorder_msgs sensor_interface proto_msgs
  INCLUDE_DIRS ${ROS_CPP_STRUCT_CONVERT_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)
