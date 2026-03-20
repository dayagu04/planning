#pragma once

#include "base_convert.h"
#include "c/fusion_spatial_parking_slot_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FusionSpatialParkingSlotInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.slot_side, ros_v.slot_side, type);
  for (size_t i0 = 0; i0 < ros_v.local_corner_point.size(); i0++) {
	  convert(struct_v.local_corner_point[i0], ros_v.local_corner_point[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.global_corner_point.size(); i1++) {
	  convert(struct_v.global_corner_point[i1], ros_v.global_corner_point[i1], type);
  }
  convert(struct_v.theta, ros_v.theta, type);
  convert(struct_v.slot_type, ros_v.slot_type, type);
  convert(struct_v.slot_update_flag, ros_v.slot_update_flag, type);
  convert(struct_v.is_free_slot, ros_v.is_free_slot, type);
  for (size_t i2 = 0; i2 < ros_v.free_slot_collision_quadrant.size(); i2++) {
	  convert(struct_v.free_slot_collision_quadrant[i2], ros_v.free_slot_collision_quadrant[i2], type);
  }
  convert(struct_v.is_one_touch_park_side, ros_v.is_one_touch_park_side, type);
}

template <typename T2>
void convert(iflyauto::FusionSpatialParkingSlot &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.spatial_slots_size, ros_v.spatial_slots_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.spatial_slots_size >= 0 && struct_v.spatial_slots_size <= FUSION_SPATIAL_PARKING_SLOT_MAX_NUM) {
      ros_v.spatial_slots.resize(struct_v.spatial_slots_size);
    } else {
      std::cout << "convert/fusion_spatial_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] spatial_slots_size=" << struct_v.spatial_slots_size 
                << " not in range FUSION_SPATIAL_PARKING_SLOT_MAX_NUM=" << FUSION_SPATIAL_PARKING_SLOT_MAX_NUM 
                << std::endl;
      ros_v.spatial_slots_size = FUSION_SPATIAL_PARKING_SLOT_MAX_NUM;
      ros_v.spatial_slots.resize(FUSION_SPATIAL_PARKING_SLOT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.spatial_slots.size(); i0++) {
      convert(struct_v.spatial_slots[i0], ros_v.spatial_slots[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.spatial_slots_size > FUSION_SPATIAL_PARKING_SLOT_MAX_NUM || ros_v.spatial_slots_size < 0 || ros_v.spatial_slots.size() > FUSION_SPATIAL_PARKING_SLOT_MAX_NUM) {
      std::cout << "convert/fusion_spatial_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] spatial_slots_size=" << ros_v.spatial_slots_size 
                << " ros_v.spatial_slots.size()=" << ros_v.spatial_slots.size()
                << " not in range FUSION_SPATIAL_PARKING_SLOT_MAX_NUM=" << FUSION_SPATIAL_PARKING_SLOT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.spatial_slots.size() > FUSION_SPATIAL_PARKING_SLOT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_SPATIAL_PARKING_SLOT_MAX_NUM; i0++) {
        convert(struct_v.spatial_slots[i0], ros_v.spatial_slots[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.spatial_slots.size(); i0++) {
        convert(struct_v.spatial_slots[i0], ros_v.spatial_slots[i0], type);
      }
    }
  }
  //
  convert(struct_v.is_near_obstacle_park_side, ros_v.is_near_obstacle_park_side, type);
}

