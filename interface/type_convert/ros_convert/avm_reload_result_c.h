#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AvmReloadResult.h"
#include "struct_msgs_v2_10/AvmReloadResult.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AvmReloadResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.reload_state, ros_v.reload_state, type);
}

