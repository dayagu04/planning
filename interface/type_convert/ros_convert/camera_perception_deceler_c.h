#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/Deceler.h"
#include "struct_msgs_v2_10/Deceler.h"
#include "struct_msgs/DecelerPerceptionInfo.h"
#include "struct_msgs_v2_10/DecelerPerceptionInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Deceler &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.deceler_points[i], ros_v.deceler_points[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.life_time, ros_v.life_time, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::DecelerPerceptionInfo &ros_v, ConvertTypeInfo type) {
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

REG_CONVERT_SINGLE(_iflytek_camera_perception_deceler_converter, "/iflytek/camera_perception/deceler", DecelerPerceptionInfo);
