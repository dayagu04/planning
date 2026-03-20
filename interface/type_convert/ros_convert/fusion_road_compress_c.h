#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/LaneBoundaryCompress.h"
#include "struct_msgs_v2_10/LaneBoundaryCompress.h"
#include "struct_msgs/RoadInfoCompress.h"
#include "struct_msgs_v2_10/RoadInfoCompress.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.existence, ros_v.existence, type);
	convert(old_ros_v.type, ros_v.type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.poly_coefficient[i], ros_v.poly_coefficient[i], type);
	}
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.line_type, ros_v.line_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadInfoCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	for (int i = 0; i < 6; i++) {
	    convert(old_ros_v.line_info[i], ros_v.line_info[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_fusion_road_fusion_compress_converter, "/iflytek/fusion/road_fusion_compress", RoadInfoCompress);
