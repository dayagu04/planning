#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ParkingFusionInfo.h"
#include "struct_msgs_v2_10/ParkingFusionInfo.h"
#include "struct_msgs/ParkingFusionLimiter.h"
#include "struct_msgs_v2_10/ParkingFusionLimiter.h"
#include "struct_msgs/ParkingFusionSlot.h"
#include "struct_msgs_v2_10/ParkingFusionSlot.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingFusionInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.parking_fusion_slot_lists_size, ros_v.parking_fusion_slot_lists_size, type);
	ros_v.parking_fusion_slot_lists.resize(old_ros_v.parking_fusion_slot_lists.size());
	for (int i = 0; i < ros_v.parking_fusion_slot_lists.size(); i++) {
	    convert(old_ros_v.parking_fusion_slot_lists[i], ros_v.parking_fusion_slot_lists[i], type);
	}
	convert(old_ros_v.select_slot_id, ros_v.select_slot_id, type);
	convert(old_ros_v.select_local_map_slot_id, ros_v.select_local_map_slot_id, type);
  convert(old_ros_v.memorized_slot_id, ros_v.memorized_slot_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingFusionLimiter &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.resource_type, ros_v.resource_type, type);
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.end_points[i], ros_v.end_points[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingFusionSlot &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.uss_id, ros_v.uss_id, type);
	convert(old_ros_v.resource_type, ros_v.resource_type, type);
	convert(old_ros_v.type, ros_v.type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.corner_points[i], ros_v.corner_points[i], type);
	}
	convert(old_ros_v.line_a, ros_v.line_a, type);
	convert(old_ros_v.line_b, ros_v.line_b, type);
	convert(old_ros_v.fusion_source, ros_v.fusion_source, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.slot_side, ros_v.slot_side, type);
	convert(old_ros_v.limiters_size, ros_v.limiters_size, type);
	ros_v.limiters.resize(old_ros_v.limiters.size());
	for (int i = 0; i < ros_v.limiters.size(); i++) {
	    convert(old_ros_v.limiters[i], ros_v.limiters[i], type);
	}
	convert(old_ros_v.allow_parking, ros_v.allow_parking, type);
	convert(old_ros_v.is_turn_corner, ros_v.is_turn_corner, type);
}

REG_CONVERT_SINGLE(_iflytek_fusion_parking_slot_converter, "/iflytek/fusion/parking_slot", ParkingFusionInfo);
