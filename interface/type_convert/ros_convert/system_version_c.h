#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/SystemVersion.h"
#include "struct_msgs_v2_10/SystemVersion.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SystemVersion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	for (int i = 0; i < 32; i++) {
	    convert(old_ros_v.system_version[i], ros_v.system_version[i], type);
	}
	static char cur_interface_version[32] = "interface2.11";
	for (int i = 0; i < 32; i++) {
	    convert(cur_interface_version[i], ros_v.interface_version[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_system_version_converter, "/iflytek/system/version", SystemVersion);
