#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionGroundLine.h"
#include "struct_msgs_v2_10/FusionGroundLine.h"
#include "struct_msgs/FusionGroundLineInfo.h"
#include "struct_msgs_v2_10/FusionGroundLineInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionGroundLine &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.resource_type, ros_v.resource_type, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.groundline_point_size, ros_v.groundline_point_size, type);
	ros_v.groundline_point.resize(old_ros_v.groundline_point.size());
	for (int i = 0; i < ros_v.groundline_point.size(); i++) {
	    convert(old_ros_v.groundline_point[i], ros_v.groundline_point[i], type);
	}
	convert(old_ros_v.reserved, ros_v.reserved, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionGroundLineInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.groundline_size, ros_v.groundline_size, type);
	ros_v.groundline.resize(old_ros_v.groundline.size());
	for (int i = 0; i < ros_v.groundline.size(); i++) {
	    convert(old_ros_v.groundline[i], ros_v.groundline[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_fusion_ground_line_converter, "/iflytek/fusion/ground_line", FusionGroundLineInfo);
