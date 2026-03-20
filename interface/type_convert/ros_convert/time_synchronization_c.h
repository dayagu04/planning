#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/TimeSynchronizationInfo.h"
#include "struct_msgs_v2_10/TimeSynchronizationInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TimeSynchronizationInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.vartualTimeStamp, ros_v.vartualTimeStamp, type);
	convert(old_ros_v.realTimeStamp, ros_v.realTimeStamp, type);
}

REG_CONVERT_SINGLE(_iflytek_other_time_synchronization_converter, "/iflytek/other/time_synchronization", TimeSynchronizationInfo);
