#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FusionSpatialParkingSlot.h"
#include "struct_msgs_v2_10/FusionSpatialParkingSlot.h"
#include "struct_msgs/FusionSpatialParkingSlotInfo.h"
#include "struct_msgs_v2_10/FusionSpatialParkingSlotInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionSpatialParkingSlot &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.spatial_slots_size, ros_v.spatial_slots_size, type);
	ros_v.spatial_slots.resize(old_ros_v.spatial_slots.size());
	for (int i = 0; i < ros_v.spatial_slots.size(); i++) {
	    convert(old_ros_v.spatial_slots[i], ros_v.spatial_slots[i], type);
	}
	convert(old_ros_v.is_near_obstacle_park_side, ros_v.is_near_obstacle_park_side, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FusionSpatialParkingSlotInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.slot_side, ros_v.slot_side, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.local_corner_point[i], ros_v.local_corner_point[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.global_corner_point[i], ros_v.global_corner_point[i], type);
	}
	convert(old_ros_v.theta, ros_v.theta, type);
	convert(old_ros_v.slot_type, ros_v.slot_type, type);
	convert(old_ros_v.slot_update_flag, ros_v.slot_update_flag, type);
	convert(old_ros_v.is_free_slot, ros_v.is_free_slot, type);
	for (int i = 0; i < 6; i++) {
	    convert(old_ros_v.free_slot_collision_quadrant[i], ros_v.free_slot_collision_quadrant[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_fusion_spatial_parking_slot_converter, "/iflytek/fusion/spatial_parking_slot", FusionSpatialParkingSlot);
