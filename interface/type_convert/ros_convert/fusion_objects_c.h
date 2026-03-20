#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionObject.h"
#include "struct_msgs_v2_10/FusionObject.h"
#include "struct_msgs/FusionObjectsAdditional.h"
#include "struct_msgs_v2_10/FusionObjectsAdditional.h"
#include "struct_msgs/FusionObjectsInfo.h"
#include "struct_msgs_v2_10/FusionObjectsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.common_info, ros_v.common_info, type);
	convert(old_ros_v.additional_info, ros_v.additional_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObjectsAdditional &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.motion_pattern_current, ros_v.motion_pattern_current, type);
	convert(old_ros_v.motion_pattern_history, ros_v.motion_pattern_history, type);
	convert(old_ros_v.fusion_source, ros_v.fusion_source, type);
	convert(old_ros_v.track_id, ros_v.track_id, type);
	convert(old_ros_v.track_age, ros_v.track_age, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.track_status, ros_v.track_status, type);
	convert(old_ros_v.bounding_box_points_size, ros_v.bounding_box_points_size, type);
	ros_v.bounding_box_points.resize(old_ros_v.bounding_box_points.size());
	for (int i = 0; i < ros_v.bounding_box_points.size(); i++) {
	    convert(old_ros_v.bounding_box_points[i], ros_v.bounding_box_points[i], type);
	}
	convert(old_ros_v.polygon_points_size, ros_v.polygon_points_size, type);
	ros_v.polygon_points.resize(old_ros_v.polygon_points.size());
	for (int i = 0; i < ros_v.polygon_points.size(); i++) {
	    convert(old_ros_v.polygon_points[i], ros_v.polygon_points[i], type);
	}
	convert(old_ros_v.relative_speed_angle, ros_v.relative_speed_angle, type);
	convert(old_ros_v.sensor_source_size, ros_v.sensor_source_size, type);
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.sensor_source_id[i], ros_v.sensor_source_id[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionObjectsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.fusion_object_size, ros_v.fusion_object_size, type);
	ros_v.fusion_object.resize(old_ros_v.fusion_object.size());
	for (int i = 0; i < ros_v.fusion_object.size(); i++) {
	    convert(old_ros_v.fusion_object[i], ros_v.fusion_object[i], type);
	}
	convert(old_ros_v.local_point_valid, ros_v.local_point_valid, type);
}

REG_CONVERT_SINGLE(_iflytek_fusion_objects_converter, "/iflytek/fusion/objects", FusionObjectsInfo);
