#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionLaneGroundMarking.h"
#include "struct_msgs_v2_10/FusionLaneGroundMarking.h"
#include "struct_msgs/FusionLineSegment.h"
#include "struct_msgs_v2_10/FusionLineSegment.h"
#include "struct_msgs/LaneBoundary.h"
#include "struct_msgs_v2_10/LaneBoundary.h"
#include "struct_msgs/LaneBoundaryColorSegment.h"
#include "struct_msgs_v2_10/LaneBoundaryColorSegment.h"
#include "struct_msgs/LaneBoundaryTypeSegment.h"
#include "struct_msgs_v2_10/LaneBoundaryTypeSegment.h"
#include "struct_msgs/LaneMergeSplitPoint.h"
#include "struct_msgs_v2_10/LaneMergeSplitPoint.h"
#include "struct_msgs/LaneNumMsg.h"
#include "struct_msgs_v2_10/LaneNumMsg.h"
#include "struct_msgs/LaneReferenceLine.h"
#include "struct_msgs_v2_10/LaneReferenceLine.h"
#include "struct_msgs/LaneSourceMsg.h"
#include "struct_msgs_v2_10/LaneSourceMsg.h"
#include "struct_msgs/ReferenceLineMsg.h"
#include "struct_msgs_v2_10/ReferenceLineMsg.h"
#include "struct_msgs/ReferencePoint.h"
#include "struct_msgs_v2_10/ReferencePoint.h"
#include "struct_msgs/RoadInfo.h"
#include "struct_msgs_v2_10/RoadInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionLaneGroundMarking &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ground_marking_points_set_size, ros_v.ground_marking_points_set_size, type);
	ros_v.ground_marking_points_set.resize(old_ros_v.ground_marking_points_set.size());
	for (int i = 0; i < ros_v.ground_marking_points_set.size(); i++) {
	    convert(old_ros_v.ground_marking_points_set[i], ros_v.ground_marking_points_set[i], type);
	}
	convert(old_ros_v.local_ground_marking_points_set_size, ros_v.local_ground_marking_points_set_size, type);
	ros_v.local_ground_marking_points_set.resize(old_ros_v.local_ground_marking_points_set.size());
	for (int i = 0; i < ros_v.local_ground_marking_points_set.size(); i++) {
	    convert(old_ros_v.local_ground_marking_points_set[i], ros_v.local_ground_marking_points_set[i], type);
	}
	convert(old_ros_v.orientation_angle, ros_v.orientation_angle, type);
	convert(old_ros_v.turn_type, ros_v.turn_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionLineSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.a0, ros_v.a0, type);
	convert(old_ros_v.a1, ros_v.a1, type);
	convert(old_ros_v.a2, ros_v.a2, type);
	convert(old_ros_v.a3, ros_v.a3, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundary &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.existence, ros_v.existence, type);
	convert(old_ros_v.life_time, ros_v.life_time, type);
	convert(old_ros_v.track_id, ros_v.track_id, type);
	convert(old_ros_v.type, ros_v.type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.poly_coefficient[i], ros_v.poly_coefficient[i], type);
	}
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.line_segments_size, ros_v.line_segments_size, type);
	ros_v.line_segments.resize(old_ros_v.line_segments.size());
	for (int i = 0; i < ros_v.line_segments.size(); i++) {
	    convert(old_ros_v.line_segments[i], ros_v.line_segments[i], type);
	}
	convert(old_ros_v.type_segments_size, ros_v.type_segments_size, type);
	ros_v.type_segments.resize(old_ros_v.type_segments.size());
	for (int i = 0; i < ros_v.type_segments.size(); i++) {
	    convert(old_ros_v.type_segments[i], ros_v.type_segments[i], type);
	}
	convert(old_ros_v.color_segments_size, ros_v.color_segments_size, type);
	ros_v.color_segments.resize(old_ros_v.color_segments.size());
	for (int i = 0; i < ros_v.color_segments.size(); i++) {
	    convert(old_ros_v.color_segments[i], ros_v.color_segments[i], type);
	}
	convert(old_ros_v.car_points_size, ros_v.car_points_size, type);
	ros_v.car_points.resize(old_ros_v.car_points.size());
	for (int i = 0; i < ros_v.car_points.size(); i++) {
	    convert(old_ros_v.car_points[i], ros_v.car_points[i], type);
	}
	convert(old_ros_v.enu_points_size, ros_v.enu_points_size, type);
	ros_v.enu_points.resize(old_ros_v.enu_points.size());
	for (int i = 0; i < ros_v.enu_points.size(); i++) {
	    convert(old_ros_v.enu_points[i], ros_v.enu_points[i], type);
	}
	convert(old_ros_v.local_points_size, ros_v.local_points_size, type);
	for (int i = 0; i < 100; i++) {
	    convert(old_ros_v.local_point[i], ros_v.local_point[i], type);
	}
	for (int i = 0; i < 100; i++) {
	    convert(old_ros_v.point_confidence_list[i], ros_v.point_confidence_list[i], type);
	}
	convert(old_ros_v.line_width, ros_v.line_width, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryColorSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.color, ros_v.color, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryTypeSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.type, ros_v.type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneMergeSplitPoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.existence, ros_v.existence, type);
	convert(old_ros_v.merge_split_point_data_size, ros_v.merge_split_point_data_size, type);
	ros_v.merge_split_point_data.resize(old_ros_v.merge_split_point_data.size());
	for (int i = 0; i < ros_v.merge_split_point_data.size(); i++) {
	    convert(old_ros_v.merge_split_point_data[i], ros_v.merge_split_point_data[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneNumMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.left_lane_num, ros_v.left_lane_num, type);
	convert(old_ros_v.right_lane_num, ros_v.right_lane_num, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneReferenceLine &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.virtual_lane_refline_points_size, ros_v.virtual_lane_refline_points_size, type);
	ros_v.virtual_lane_refline_points.resize(old_ros_v.virtual_lane_refline_points.size());
	for (int i = 0; i < ros_v.virtual_lane_refline_points.size(); i++) {
	    convert(old_ros_v.virtual_lane_refline_points[i], ros_v.virtual_lane_refline_points[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.poly_coefficient_car[i], ros_v.poly_coefficient_car[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneSourceMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.source, ros_v.source, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ReferenceLineMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.order_id, ros_v.order_id, type);
	convert(old_ros_v.relative_id, ros_v.relative_id, type);
	convert(old_ros_v.lane_types_size, ros_v.lane_types_size, type);
	ros_v.lane_types.resize(old_ros_v.lane_types.size());
	for (int i = 0; i < ros_v.lane_types.size(); i++) {
	    convert(old_ros_v.lane_types[i], ros_v.lane_types[i], type);
	}
	convert(old_ros_v.lane_marks_size, ros_v.lane_marks_size, type);
	ros_v.lane_marks.resize(old_ros_v.lane_marks.size());
	for (int i = 0; i < ros_v.lane_marks.size(); i++) {
	    convert(old_ros_v.lane_marks[i], ros_v.lane_marks[i], type);
	}
	convert(old_ros_v.lane_sources_size, ros_v.lane_sources_size, type);
	ros_v.lane_sources.resize(old_ros_v.lane_sources.size());
	for (int i = 0; i < ros_v.lane_sources.size(); i++) {
	    convert(old_ros_v.lane_sources[i], ros_v.lane_sources[i], type);
	}
	convert(old_ros_v.lane_num_size, ros_v.lane_num_size, type);
	ros_v.lane_num.resize(old_ros_v.lane_num.size());
	for (int i = 0; i < ros_v.lane_num.size(); i++) {
	    convert(old_ros_v.lane_num[i], ros_v.lane_num[i], type);
	}
	convert(old_ros_v.lane_reference_line, ros_v.lane_reference_line, type);
	convert(old_ros_v.lane_merge_split_point, ros_v.lane_merge_split_point, type);
	convert(old_ros_v.left_lane_boundary, ros_v.left_lane_boundary, type);
	convert(old_ros_v.right_lane_boundary, ros_v.right_lane_boundary, type);
	convert(old_ros_v.stop_line, ros_v.stop_line, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ReferencePoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.track_id, ros_v.track_id, type);
	convert(old_ros_v.car_point, ros_v.car_point, type);
	convert(old_ros_v.enu_point, ros_v.enu_point, type);
	convert(old_ros_v.local_point, ros_v.local_point, type);
	convert(old_ros_v.curvature, ros_v.curvature, type);
	convert(old_ros_v.car_heading, ros_v.car_heading, type);
	convert(old_ros_v.enu_heading, ros_v.enu_heading, type);
	convert(old_ros_v.local_heading, ros_v.local_heading, type);
	convert(old_ros_v.distance_to_left_road_border, ros_v.distance_to_left_road_border, type);
	convert(old_ros_v.distance_to_right_road_border, ros_v.distance_to_right_road_border, type);
	convert(old_ros_v.distance_to_left_lane_border, ros_v.distance_to_left_lane_border, type);
	convert(old_ros_v.distance_to_right_lane_border, ros_v.distance_to_right_lane_border, type);
	convert(old_ros_v.lane_width, ros_v.lane_width, type);
	convert(old_ros_v.speed_limit_max, ros_v.speed_limit_max, type);
	convert(old_ros_v.speed_limit_min, ros_v.speed_limit_min, type);
	convert(old_ros_v.left_road_border_type, ros_v.left_road_border_type, type);
	convert(old_ros_v.right_road_border_type, ros_v.right_road_border_type, type);
	convert(old_ros_v.left_lane_border_type, ros_v.left_lane_border_type, type);
	convert(old_ros_v.right_lane_border_type, ros_v.right_lane_border_type, type);
	convert(old_ros_v.is_in_intersection, ros_v.is_in_intersection, type);
	convert(old_ros_v.lane_type, ros_v.lane_type, type);
	convert(old_ros_v.s, ros_v.s, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.reference_line_msg_size, ros_v.reference_line_msg_size, type);
	ros_v.reference_line_msg.resize(old_ros_v.reference_line_msg.size());
	for (int i = 0; i < ros_v.reference_line_msg.size(); i++) {
	    convert(old_ros_v.reference_line_msg[i], ros_v.reference_line_msg[i], type);
	}
	convert(old_ros_v.lane_ground_markings_size, ros_v.lane_ground_markings_size, type);
	ros_v.lane_ground_markings.resize(old_ros_v.lane_ground_markings.size());
	for (int i = 0; i < ros_v.lane_ground_markings.size(); i++) {
	    convert(old_ros_v.lane_ground_markings[i], ros_v.lane_ground_markings[i], type);
	}
	convert(old_ros_v.local_point_valid, ros_v.local_point_valid, type);
}

REG_CONVERT_SINGLE(_iflytek_fusion_road_fusion_converter, "/iflytek/fusion/road_fusion", RoadInfo);
