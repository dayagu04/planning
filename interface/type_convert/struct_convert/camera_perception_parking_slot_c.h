#pragma once

#include "base_convert.h"
#include "c/camera_perception_parking_slot_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraSlotType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.material, ros_v.material, type);
  for (size_t i0 = 0; i0 < ros_v.corner_points.size(); i0++) {
	  convert(struct_v.corner_points[i0], ros_v.corner_points[i0], type);
  }
  convert(struct_v.line_a, ros_v.line_a, type);
  convert(struct_v.line_b, ros_v.line_b, type);
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.allow_parking, ros_v.allow_parking, type);
}

template <typename T2>
void convert(iflyauto::VisionSlotLimiter &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  for (size_t i0 = 0; i0 < ros_v.limiter_points.size(); i0++) {
	  convert(struct_v.limiter_points[i0], ros_v.limiter_points[i0], type);
  }
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::VisionSlotSpecification &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  for (size_t i0 = 0; i0 < ros_v.specificatio_points.size(); i0++) {
	  convert(struct_v.specificatio_points[i0], ros_v.specificatio_points[i0], type);
  }
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::ParkingSlotSelectInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.parking_slots_size, ros_v.parking_slots_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.parking_slots_size >= 0 && struct_v.parking_slots_size <= CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      ros_v.parking_slots.resize(struct_v.parking_slots_size);
    } else {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] parking_slots_size=" << struct_v.parking_slots_size 
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
      ros_v.parking_slots_size = CAMERA_PERCEPTION_PARKING_SLOTS_NUM;
      ros_v.parking_slots.resize(CAMERA_PERCEPTION_PARKING_SLOTS_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.parking_slots.size(); i0++) {
      convert(struct_v.parking_slots[i0], ros_v.parking_slots[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.parking_slots_size > CAMERA_PERCEPTION_PARKING_SLOTS_NUM || ros_v.parking_slots_size < 0 || ros_v.parking_slots.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] parking_slots_size=" << ros_v.parking_slots_size 
                << " ros_v.parking_slots.size()=" << ros_v.parking_slots.size()
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
    }
    if (ros_v.parking_slots.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_PARKING_SLOTS_NUM; i0++) {
        convert(struct_v.parking_slots[i0], ros_v.parking_slots[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.parking_slots.size(); i0++) {
        convert(struct_v.parking_slots[i0], ros_v.parking_slots[i0], type);
      }
    }
  }
  //
  convert(struct_v.vision_slot_limiters_size, ros_v.vision_slot_limiters_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.vision_slot_limiters_size >= 0 && struct_v.vision_slot_limiters_size <= CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      ros_v.vision_slot_limiters.resize(struct_v.vision_slot_limiters_size);
    } else {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] vision_slot_limiters_size=" << struct_v.vision_slot_limiters_size 
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
      ros_v.vision_slot_limiters_size = CAMERA_PERCEPTION_PARKING_SLOTS_NUM;
      ros_v.vision_slot_limiters.resize(CAMERA_PERCEPTION_PARKING_SLOTS_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.vision_slot_limiters.size(); i1++) {
      convert(struct_v.vision_slot_limiters[i1], ros_v.vision_slot_limiters[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.vision_slot_limiters_size > CAMERA_PERCEPTION_PARKING_SLOTS_NUM || ros_v.vision_slot_limiters_size < 0 || ros_v.vision_slot_limiters.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] vision_slot_limiters_size=" << ros_v.vision_slot_limiters_size 
                << " ros_v.vision_slot_limiters.size()=" << ros_v.vision_slot_limiters.size()
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
    }
    if (ros_v.vision_slot_limiters.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_PARKING_SLOTS_NUM; i1++) {
        convert(struct_v.vision_slot_limiters[i1], ros_v.vision_slot_limiters[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.vision_slot_limiters.size(); i1++) {
        convert(struct_v.vision_slot_limiters[i1], ros_v.vision_slot_limiters[i1], type);
      }
    }
  }
  //
  convert(struct_v.vision_slot_specificatios_size, ros_v.vision_slot_specificatios_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.vision_slot_specificatios_size >= 0 && struct_v.vision_slot_specificatios_size <= CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      ros_v.vision_slot_specificatios.resize(struct_v.vision_slot_specificatios_size);
    } else {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << " [convert][TO_ROS] vision_slot_specificatios_size=" << struct_v.vision_slot_specificatios_size 
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
      ros_v.vision_slot_specificatios_size = CAMERA_PERCEPTION_PARKING_SLOTS_NUM;
      ros_v.vision_slot_specificatios.resize(CAMERA_PERCEPTION_PARKING_SLOTS_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.vision_slot_specificatios.size(); i2++) {
      convert(struct_v.vision_slot_specificatios[i2], ros_v.vision_slot_specificatios[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.vision_slot_specificatios_size > CAMERA_PERCEPTION_PARKING_SLOTS_NUM || ros_v.vision_slot_specificatios_size < 0 || ros_v.vision_slot_specificatios.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      std::cout << "convert/camera_perception_parking_slot_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] vision_slot_specificatios_size=" << ros_v.vision_slot_specificatios_size 
                << " ros_v.vision_slot_specificatios.size()=" << ros_v.vision_slot_specificatios.size()
                << " not in range CAMERA_PERCEPTION_PARKING_SLOTS_NUM=" << CAMERA_PERCEPTION_PARKING_SLOTS_NUM 
                << std::endl;
    }
    if (ros_v.vision_slot_specificatios.size() > CAMERA_PERCEPTION_PARKING_SLOTS_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_PARKING_SLOTS_NUM; i2++) {
        convert(struct_v.vision_slot_specificatios[i2], ros_v.vision_slot_specificatios[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.vision_slot_specificatios.size(); i2++) {
        convert(struct_v.vision_slot_specificatios[i2], ros_v.vision_slot_specificatios[i2], type);
      }
    }
  }
  //
  for (size_t i3 = 0; i3 < ros_v.ego_motion.size(); i3++) {
	  convert(struct_v.ego_motion[i3], ros_v.ego_motion[i3], type);
  }
  convert(struct_v.is_after_reset, ros_v.is_after_reset, type);
  for (size_t i4 = 0; i4 < ros_v.images_timestamp.size(); i4++) {
	  convert(struct_v.images_timestamp[i4], ros_v.images_timestamp[i4], type);
  }
}

