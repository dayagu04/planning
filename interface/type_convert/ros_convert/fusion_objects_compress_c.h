#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionObjectCompress.h"
#include "struct_msgs_v2_10/FusionObjectCompress.h"
#include "struct_msgs/FusionObjectsAdditionalCompress.h"
#include "struct_msgs_v2_10/FusionObjectsAdditionalCompress.h"
#include "struct_msgs/FusionObjectsInfoCompress.h"
#include "struct_msgs_v2_10/FusionObjectsInfoCompress.h"
#include "struct_msgs/ObstacleCompress.h"
#include "struct_msgs_v2_10/ObstacleCompress.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObjectCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.common_info, ros_v.common_info, type);
	convert(old_ros_v.additional_info, ros_v.additional_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObjectsAdditionalCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fusion_source, ros_v.fusion_source, type);
	convert(old_ros_v.track_id, ros_v.track_id, type);
	convert(old_ros_v.track_age, ros_v.track_age, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObjectsInfoCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.fusion_object_size, ros_v.fusion_object_size, type);
	ros_v.fusion_object.resize(old_ros_v.fusion_object.size());
	for (int i = 0; i < ros_v.fusion_object.size(); i++) {
	    convert(old_ros_v.fusion_object[i], ros_v.fusion_object[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ObstacleCompress &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.shape, ros_v.shape, type);
	convert(old_ros_v.relative_velocity, ros_v.relative_velocity, type);
	convert(old_ros_v.relative_acceleration, ros_v.relative_acceleration, type);
	convert(old_ros_v.relative_center_position, ros_v.relative_center_position, type);
	convert(old_ros_v.relative_heading_angle, ros_v.relative_heading_angle, type);
	convert(old_ros_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
}

REG_CONVERT_SINGLE(_iflytek_fusion_objects_compress_converter, "/iflytek/fusion/objects_compress", FusionObjectsInfoCompress);
