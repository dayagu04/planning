#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/GroundLine.h"
#include "struct_msgs_v2_10/GroundLine.h"
#include "struct_msgs/GroundLinePerceptionInfo.h"
#include "struct_msgs_v2_10/GroundLinePerceptionInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::GroundLine &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.camera_source, ros_v.camera_source, type);
	convert(old_ros_v.camera_model, ros_v.camera_model, type);
	convert(old_ros_v.semantic_type, ros_v.semantic_type, type);
	convert(old_ros_v.point_type, ros_v.point_type, type);
	convert(old_ros_v.visable_seg_num, ros_v.visable_seg_num, type);
	convert(old_ros_v.points_2d_size, ros_v.points_2d_size, type);
	convert(old_ros_v.points_3d_size, ros_v.points_3d_size, type);
	ros_v.points_2d.resize(old_ros_v.points_2d.size());
	for (int i = 0; i < ros_v.points_2d.size(); i++) {
	    convert(old_ros_v.points_2d[i], ros_v.points_2d[i], type);
	}
	ros_v.points_3d.resize(old_ros_v.points_3d.size());
	for (int i = 0; i < ros_v.points_3d.size(); i++) {
	    convert(old_ros_v.points_3d[i], ros_v.points_3d[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::GroundLinePerceptionInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.ground_lines_size, ros_v.ground_lines_size, type);
	ros_v.ground_lines.resize(old_ros_v.ground_lines.size());
	for (int i = 0; i < ros_v.ground_lines.size(); i++) {
	    convert(old_ros_v.ground_lines[i], ros_v.ground_lines[i], type);
	}
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.reserved_infos[i], ros_v.reserved_infos[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_ground_line_converter, "/iflytek/camera_perception/ground_line", GroundLinePerceptionInfo);
