#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionDeceler.h"
#include "struct_msgs_v2_10/FusionDeceler.h"
#include "struct_msgs/FusionDecelerInfo.h"
#include "struct_msgs_v2_10/FusionDecelerInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionDeceler &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.resource_type, ros_v.resource_type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.deceler_points[i], ros_v.deceler_points[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.reserved, ros_v.reserved, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionDecelerInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.decelers_size, ros_v.decelers_size, type);
	ros_v.decelers.resize(old_ros_v.decelers.size());
	for (int i = 0; i < ros_v.decelers.size(); i++) {
	    convert(old_ros_v.decelers[i], ros_v.decelers[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_fusion_speed_bump_converter, "/iflytek/fusion/speed_bump", FusionDecelerInfo);
