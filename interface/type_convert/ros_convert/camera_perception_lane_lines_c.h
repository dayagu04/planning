#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ColorSegment.h"
#include "struct_msgs_v2_10/ColorSegment.h"
#include "struct_msgs/LaneData.h"
#include "struct_msgs_v2_10/LaneData.h"
#include "struct_msgs/LaneGroundMarking.h"
#include "struct_msgs_v2_10/LaneGroundMarking.h"
#include "struct_msgs/LaneLine.h"
#include "struct_msgs_v2_10/LaneLine.h"
#include "struct_msgs/LaneLineSet.h"
#include "struct_msgs_v2_10/LaneLineSet.h"
#include "struct_msgs/LaneMatchFlag.h"
#include "struct_msgs_v2_10/LaneMatchFlag.h"
#include "struct_msgs/LanePointAttr.h"
#include "struct_msgs_v2_10/LanePointAttr.h"
#include "struct_msgs/LanePositionSeg.h"
#include "struct_msgs_v2_10/LanePositionSeg.h"
#include "struct_msgs/LineSegment.h"
#include "struct_msgs_v2_10/LineSegment.h"
#include "struct_msgs/MarkingSegment.h"
#include "struct_msgs_v2_10/MarkingSegment.h"
#include "struct_msgs/SpeedInfo.h"
#include "struct_msgs_v2_10/SpeedInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ColorSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.color, ros_v.color, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.order_id, ros_v.order_id, type);
	convert(old_ros_v.central_line, ros_v.central_line, type);
	convert(old_ros_v.lane_types_size, ros_v.lane_types_size, type);
	ros_v.lane_types.resize(old_ros_v.lane_types.size());
	for (int i = 0; i < ros_v.lane_types.size(); i++) {
	    convert(old_ros_v.lane_types[i], ros_v.lane_types[i], type);
	}
	convert(old_ros_v.turn_types_size, ros_v.turn_types_size, type);
	ros_v.turn_types.resize(old_ros_v.turn_types.size());
	for (int i = 0; i < ros_v.turn_types.size(); i++) {
	    convert(old_ros_v.turn_types[i], ros_v.turn_types[i], type);
	}
	convert(old_ros_v.position_segs_size, ros_v.position_segs_size, type);
	ros_v.position_segs.resize(old_ros_v.position_segs.size());
	for (int i = 0; i < ros_v.position_segs.size(); i++) {
	    convert(old_ros_v.position_segs[i], ros_v.position_segs[i], type);
	}
	convert(old_ros_v.match_flags_size, ros_v.match_flags_size, type);
	ros_v.match_flags.resize(old_ros_v.match_flags.size());
	for (int i = 0; i < ros_v.match_flags.size(); i++) {
	    convert(old_ros_v.match_flags[i], ros_v.match_flags[i], type);
	}
	convert(old_ros_v.left_lane_boundary_id, ros_v.left_lane_boundary_id, type);
	convert(old_ros_v.right_lane_boundary_id, ros_v.right_lane_boundary_id, type);
	convert(old_ros_v.left_road_boundary_id, ros_v.left_road_boundary_id, type);
	convert(old_ros_v.right_road_boundary_id, ros_v.right_road_boundary_id, type);
	convert(old_ros_v.stop_line_id, ros_v.stop_line_id, type);
	convert(old_ros_v.merge_split_points_size, ros_v.merge_split_points_size, type);
	ros_v.merge_split_points.resize(old_ros_v.merge_split_points.size());
	for (int i = 0; i < ros_v.merge_split_points.size(); i++) {
	    convert(old_ros_v.merge_split_points[i], ros_v.merge_split_points[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneGroundMarking &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ground_marking_points_set_size, ros_v.ground_marking_points_set_size, type);
	ros_v.ground_marking_points_set.resize(old_ros_v.ground_marking_points_set.size());
	for (int i = 0; i < ros_v.ground_marking_points_set.size(); i++) {
	    convert(old_ros_v.ground_marking_points_set[i], ros_v.ground_marking_points_set[i], type);
	}
	convert(old_ros_v.orientation_angle, ros_v.orientation_angle, type);
	convert(old_ros_v.turn_type, ros_v.turn_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneLine &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.life_time, ros_v.life_time, type);
	convert(old_ros_v.source, ros_v.source, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.pos_type, ros_v.pos_type, type);
	convert(old_ros_v.lane_points_attr_set_size, ros_v.lane_points_attr_set_size, type);
	ros_v.lane_points_attr_set.resize(old_ros_v.lane_points_attr_set.size());
	for (int i = 0; i < ros_v.lane_points_attr_set.size(); i++) {
	    convert(old_ros_v.lane_points_attr_set[i], ros_v.lane_points_attr_set[i], type);
	}
	convert(old_ros_v.line_segments_size, ros_v.line_segments_size, type);
	ros_v.line_segments.resize(old_ros_v.line_segments.size());
	for (int i = 0; i < ros_v.line_segments.size(); i++) {
	    convert(old_ros_v.line_segments[i], ros_v.line_segments[i], type);
	}
	convert(old_ros_v.marking_segments_size, ros_v.marking_segments_size, type);
	ros_v.marking_segments.resize(old_ros_v.marking_segments.size());
	for (int i = 0; i < ros_v.marking_segments.size(); i++) {
	    convert(old_ros_v.marking_segments[i], ros_v.marking_segments[i], type);
	}
	convert(old_ros_v.color_segments_size, ros_v.color_segments_size, type);
	ros_v.color_segments.resize(old_ros_v.color_segments.size());
	for (int i = 0; i < ros_v.color_segments.size(); i++) {
	    convert(old_ros_v.color_segments[i], ros_v.color_segments[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneLineSet &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.lanes_size, ros_v.lanes_size, type);
	ros_v.lanes.resize(old_ros_v.lanes.size());
	for (int i = 0; i < ros_v.lanes.size(); i++) {
	    convert(old_ros_v.lanes[i], ros_v.lanes[i], type);
	}
	convert(old_ros_v.lane_line_size, ros_v.lane_line_size, type);
	ros_v.lane_line.resize(old_ros_v.lane_line.size());
	for (int i = 0; i < ros_v.lane_line.size(); i++) {
	    convert(old_ros_v.lane_line[i], ros_v.lane_line[i], type);
	}
	convert(old_ros_v.lane_ground_markings_size, ros_v.lane_ground_markings_size, type);
	ros_v.lane_ground_markings.resize(old_ros_v.lane_ground_markings.size());
	for (int i = 0; i < ros_v.lane_ground_markings.size(); i++) {
	    convert(old_ros_v.lane_ground_markings[i], ros_v.lane_ground_markings[i], type);
	}
	convert(old_ros_v.stop_line_size, ros_v.stop_line_size, type);
	ros_v.stop_line.resize(old_ros_v.stop_line.size());
	for (int i = 0; i < ros_v.stop_line.size(); i++) {
	    convert(old_ros_v.stop_line[i], ros_v.stop_line[i], type);
	}
	convert(old_ros_v.inhibit_line_size, ros_v.inhibit_line_size, type);
	ros_v.inhibit_line.resize(old_ros_v.inhibit_line.size());
	for (int i = 0; i < ros_v.inhibit_line.size(); i++) {
	    convert(old_ros_v.inhibit_line[i], ros_v.inhibit_line[i], type);
	}
	convert(old_ros_v.speed_info_size, ros_v.speed_info_size, type);
	ros_v.speed_info.resize(old_ros_v.speed_info.size());
	for (int i = 0; i < ros_v.speed_info.size(); i++) {
	    convert(old_ros_v.speed_info[i], ros_v.speed_info[i], type);
	}
	convert(old_ros_v.camera_perception_input_timestamp, ros_v.camera_perception_input_timestamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneMatchFlag &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.match_current_lane, ros_v.match_current_lane, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LanePointAttr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_point_marking, ros_v.lane_point_marking, type);
	convert(old_ros_v.lane_point_color, ros_v.lane_point_color, type);
	convert(old_ros_v.lane_point_confidence, ros_v.lane_point_confidence, type);
	convert(old_ros_v.lane_point_coordinate, ros_v.lane_point_coordinate, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LanePositionSeg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.left_to_right_id, ros_v.left_to_right_id, type);
	convert(old_ros_v.right_to_left_id, ros_v.right_to_left_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LineSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.a0, ros_v.a0, type);
	convert(old_ros_v.a1, ros_v.a1, type);
	convert(old_ros_v.a2, ros_v.a2, type);
	convert(old_ros_v.a3, ros_v.a3, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::MarkingSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.marking, ros_v.marking, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SpeedInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.speed_limit, ros_v.speed_limit, type);
	convert(old_ros_v.lane_assignment, ros_v.lane_assignment, type);
	convert(old_ros_v.source, ros_v.source, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_parking_lane_line_converter, "/iflytek/camera_perception/parking_lane_line", LaneLineSet);
REG_CONVERT_SINGLE(_iflytek_camera_perception_lane_lines_converter, "/iflytek/camera_perception/lane_lines", LaneLineSet);
REG_CONVERT_SINGLE(_iflytek_camera_perception_lane_topo_converter, "/iflytek/camera_perception/lane_topo", LaneLineSet);
REG_CONVERT_SINGLE(_iflytek_camera_perception_lane_lines_debug_info_converter, "/iflytek/camera_perception/lane_lines_debug_info", LaneLineSet);
REG_CONVERT_SINGLE(_iflytek_camera_perception_lane_topo_debug_info_converter, "/iflytek/camera_perception/lane_topo_debug_info", LaneLineSet);
REG_CONVERT_SINGLE(_mobileye_camera_perception_lane_lines_converter, "/mobileye/camera_perception/lane_lines", LaneLineSet);
