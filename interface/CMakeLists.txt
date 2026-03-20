cmake_minimum_required (VERSION 2.8)

project (AutonomousDriving)

# proto
add_subdirectory (src/c EXCLUDE_FROM_ALL)
add_subdirectory (src/proto EXCLUDE_FROM_ALL)
add_subdirectory (src/legacy EXCLUDE_FROM_ALL)

# test
add_subdirectory (test EXCLUDE_FROM_ALL)

if(INTERFACE_COMPILE_MODE STREQUAL "check_type_convert")
    # # rosmsg generation 'make -j'
    add_subdirectory (rosmsg/proto_msgs)
    add_subdirectory (rosmsg/struct_msgs)
    add_subdirectory (rosmsg/sensor_interface)
    add_subdirectory (type_convert/struct_convert)

    # struct convert legacy
    add_subdirectory (rosmsg/struct_msgs_legacy/interface2.4.5)
    add_subdirectory (rosmsg/struct_msgs_legacy/interface2.4.6)
    add_subdirectory (type_convert/struct_convert_legacy)

    # # rosbag converter 'make rosbag_converter'
    add_subdirectory (rosmsg/struct_msgs_v2_10)
    add_subdirectory (type_convert/ros_convert)
    add_subdirectory (type_convert/rosbag_converter)
endif()

if(ADAPTER STREQUAL "simulation")
    add_subdirectory (rosmsg/proto_msgs)
    add_subdirectory (rosmsg/struct_msgs)
    add_subdirectory (rosmsg/sensor_interface)
    add_subdirectory (rosmsg/struct_msgs_legacy/interface2.4.5)
    add_subdirectory (rosmsg/struct_msgs_legacy/interface2.4.6)
    add_subdirectory (type_convert/struct_convert)
    add_subdirectory (type_convert/struct_convert_legacy)
endif()
