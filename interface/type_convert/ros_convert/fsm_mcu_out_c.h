#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FsmMcuOut.h"
#include "struct_msgs_v2_10/FsmMcuOut.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FsmMcuOut &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.mcu_state, ros_v.mcu_state, type);
	convert(old_ros_v.lat_override_apa, ros_v.lat_override_apa, type);
	convert(old_ros_v.lat_override_pilot, ros_v.lat_override_pilot, type);
}

