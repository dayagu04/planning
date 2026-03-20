#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/MappingStatusInfo.h"
#include "struct_msgs_v2_10/MappingStatusInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::MappingStatusInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.driving_distance, ros_v.driving_distance, type);
	convert(old_ros_v.backward_distance, ros_v.backward_distance, type);
	convert(old_ros_v.saved_progress, ros_v.saved_progress, type);
	convert(old_ros_v.running_status, ros_v.running_status, type);
	convert(old_ros_v.failed_reason, ros_v.failed_reason, type);
	convert(old_ros_v.is_in_parking_slot, ros_v.is_in_parking_slot, type);
}

REG_CONVERT_SINGLE(_iflytek_mega_mapping_status_converter, "/iflytek/mega/mapping_status", MappingStatusInfo);
