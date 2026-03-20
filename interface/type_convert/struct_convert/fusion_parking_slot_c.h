#pragma once

#include "base_convert.h"
#include "c/fusion_parking_slot_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ParkingFusionLimiter &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.resource_type, ros_v.resource_type, type);
  for (size_t i0 = 0; i0 < ros_v.end_points.size(); i0++) {
	  convert(struct_v.end_points[i0], ros_v.end_points[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::ParkingFusionSlot &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.uss_id, ros_v.uss_id, type);
  convert(struct_v.resource_type, ros_v.resource_type, type);
  convert(struct_v.type, ros_v.type, type);
  for (size_t i0 = 0; i0 < ros_v.corner_points.size(); i0++) {
	  convert(struct_v.corner_points[i0], ros_v.corner_points[i0], type);
  }
  convert(struct_v.line_a, ros_v.line_a, type);
  convert(struct_v.line_b, ros_v.line_b, type);
  convert(struct_v.fusion_source, ros_v.fusion_source, type);
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.slot_side, ros_v.slot_side, type);
  convert(struct_v.limiters_size, ros_v.limiters_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.limiters_size >= 0 && struct_v.limiters_size <= FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      ros_v.limiters.resize(struct_v.limiters_size);
    } else {
      std::cout << "convert/fusion_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] limiters_size=" << struct_v.limiters_size 
                << " not in range FUSION_PARKING_SLOT_LIMITER_MAX_NUM=" << FUSION_PARKING_SLOT_LIMITER_MAX_NUM 
                << std::endl;
      ros_v.limiters_size = FUSION_PARKING_SLOT_LIMITER_MAX_NUM;
      ros_v.limiters.resize(FUSION_PARKING_SLOT_LIMITER_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.limiters.size(); i1++) {
      convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.limiters_size > FUSION_PARKING_SLOT_LIMITER_MAX_NUM || ros_v.limiters_size < 0 || ros_v.limiters.size() > FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      std::cout << "convert/fusion_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] limiters_size=" << ros_v.limiters_size 
                << " ros_v.limiters.size()=" << ros_v.limiters.size()
                << " not in range FUSION_PARKING_SLOT_LIMITER_MAX_NUM=" << FUSION_PARKING_SLOT_LIMITER_MAX_NUM 
                << std::endl;
    }
    if (ros_v.limiters.size() > FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_PARKING_SLOT_LIMITER_MAX_NUM; i1++) {
        convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.limiters.size(); i1++) {
        convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
      }
    }
  }
  //
  convert(struct_v.allow_parking, ros_v.allow_parking, type);
  convert(struct_v.is_turn_corner, ros_v.is_turn_corner, type);
}

template <typename T2>
void convert(iflyauto::ParkingFusionInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.parking_fusion_slot_lists_size, ros_v.parking_fusion_slot_lists_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.parking_fusion_slot_lists_size >= 0 && struct_v.parking_fusion_slot_lists_size <= FUSION_PARKING_SLOT_MAX_NUM) {
      ros_v.parking_fusion_slot_lists.resize(struct_v.parking_fusion_slot_lists_size);
    } else {
      std::cout << "convert/fusion_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] parking_fusion_slot_lists_size=" << struct_v.parking_fusion_slot_lists_size 
                << " not in range FUSION_PARKING_SLOT_MAX_NUM=" << FUSION_PARKING_SLOT_MAX_NUM 
                << std::endl;
      ros_v.parking_fusion_slot_lists_size = FUSION_PARKING_SLOT_MAX_NUM;
      ros_v.parking_fusion_slot_lists.resize(FUSION_PARKING_SLOT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.parking_fusion_slot_lists.size(); i0++) {
      convert(struct_v.parking_fusion_slot_lists[i0], ros_v.parking_fusion_slot_lists[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.parking_fusion_slot_lists_size > FUSION_PARKING_SLOT_MAX_NUM || ros_v.parking_fusion_slot_lists_size < 0 || ros_v.parking_fusion_slot_lists.size() > FUSION_PARKING_SLOT_MAX_NUM) {
      std::cout << "convert/fusion_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] parking_fusion_slot_lists_size=" << ros_v.parking_fusion_slot_lists_size 
                << " ros_v.parking_fusion_slot_lists.size()=" << ros_v.parking_fusion_slot_lists.size()
                << " not in range FUSION_PARKING_SLOT_MAX_NUM=" << FUSION_PARKING_SLOT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.parking_fusion_slot_lists.size() > FUSION_PARKING_SLOT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_PARKING_SLOT_MAX_NUM; i0++) {
        convert(struct_v.parking_fusion_slot_lists[i0], ros_v.parking_fusion_slot_lists[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.parking_fusion_slot_lists.size(); i0++) {
        convert(struct_v.parking_fusion_slot_lists[i0], ros_v.parking_fusion_slot_lists[i0], type);
      }
    }
  }
  //
  convert(struct_v.select_slot_id, ros_v.select_slot_id, type);
  convert(struct_v.select_local_map_slot_id, ros_v.select_local_map_slot_id, type);
  convert(struct_v.memorized_slot_id, ros_v.memorized_slot_id, type);
  convert(struct_v.is_in_parking_slot, ros_v.is_in_parking_slot, type);
  convert(struct_v.ego_slot_id, ros_v.ego_slot_id, type);
}

