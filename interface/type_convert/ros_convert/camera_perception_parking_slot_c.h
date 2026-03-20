#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraSlotType.h"
#include "struct_msgs_v2_10/CameraSlotType.h"
#include "struct_msgs/ParkingSlotSelectInfo.h"
#include "struct_msgs_v2_10/ParkingSlotSelectInfo.h"
#include "struct_msgs/VisionSlotLimiter.h"
#include "struct_msgs_v2_10/VisionSlotLimiter.h"
#include "struct_msgs/VisionSlotSpecification.h"
#include "struct_msgs_v2_10/VisionSlotSpecification.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraSlotType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.material, ros_v.material, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.corner_points[i], ros_v.corner_points[i], type);
	}
	convert(old_ros_v.line_a, ros_v.line_a, type);
	convert(old_ros_v.line_b, ros_v.line_b, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
	convert(old_ros_v.allow_parking, ros_v.allow_parking, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingSlotSelectInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.parking_slots_size, ros_v.parking_slots_size, type);
	ros_v.parking_slots.resize(old_ros_v.parking_slots.size());
	for (int i = 0; i < ros_v.parking_slots.size(); i++) {
	    convert(old_ros_v.parking_slots[i], ros_v.parking_slots[i], type);
	}
	convert(old_ros_v.vision_slot_limiters_size, ros_v.vision_slot_limiters_size, type);
	ros_v.vision_slot_limiters.resize(old_ros_v.vision_slot_limiters.size());
	for (int i = 0; i < ros_v.vision_slot_limiters.size(); i++) {
	    convert(old_ros_v.vision_slot_limiters[i], ros_v.vision_slot_limiters[i], type);
	}
	convert(old_ros_v.vision_slot_specificatios_size, ros_v.vision_slot_specificatios_size, type);
	ros_v.vision_slot_specificatios.resize(old_ros_v.vision_slot_specificatios.size());
	for (int i = 0; i < ros_v.vision_slot_specificatios.size(); i++) {
	    convert(old_ros_v.vision_slot_specificatios[i], ros_v.vision_slot_specificatios[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::VisionSlotLimiter &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.limiter_points[i], ros_v.limiter_points[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::VisionSlotSpecification &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.specificatio_points[i], ros_v.specificatio_points[i], type);
	}
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_parking_slot_list_converter, "/iflytek/camera_perception/parking_slot_list", ParkingSlotSelectInfo);
