#pragma once

#include "base_convert.h"
#include "c/fusion_objects_compress_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ObstacleCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_acceleration, ros_v.relative_acceleration, type);
  convert(struct_v.relative_center_position, ros_v.relative_center_position, type);
  convert(struct_v.relative_heading_angle, ros_v.relative_heading_angle, type);
  convert(struct_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
}

template <typename T2>
void convert(iflyauto::FusionObjectsAdditionalCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fusion_source, ros_v.fusion_source, type);
  convert(struct_v.track_id, ros_v.track_id, type);
  convert(struct_v.track_age, ros_v.track_age, type);
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::FusionObjectCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.common_info, ros_v.common_info, type);
  convert(struct_v.additional_info, ros_v.additional_info, type);
}

template <typename T2>
void convert(iflyauto::FusionObjectsInfoCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.fusion_object_size, ros_v.fusion_object_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.fusion_object_size >= 0 && struct_v.fusion_object_size <= FUSION_COMPRESS_OBJECT_MAX_NUM) {
      ros_v.fusion_object.resize(struct_v.fusion_object_size);
    } else {
      std::cout << "convert/fusion_objects_compress_c.h:" << __LINE__ 
                << " [convert][TO_ROS] fusion_object_size=" << struct_v.fusion_object_size 
                << " not in range FUSION_COMPRESS_OBJECT_MAX_NUM=" << FUSION_COMPRESS_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.fusion_object_size = FUSION_COMPRESS_OBJECT_MAX_NUM;
      ros_v.fusion_object.resize(FUSION_COMPRESS_OBJECT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.fusion_object.size(); i0++) {
      convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.fusion_object_size > FUSION_COMPRESS_OBJECT_MAX_NUM || ros_v.fusion_object_size < 0 || ros_v.fusion_object.size() > FUSION_COMPRESS_OBJECT_MAX_NUM) {
      std::cout << "convert/fusion_objects_compress_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] fusion_object_size=" << ros_v.fusion_object_size 
                << " ros_v.fusion_object.size()=" << ros_v.fusion_object.size()
                << " not in range FUSION_COMPRESS_OBJECT_MAX_NUM=" << FUSION_COMPRESS_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.fusion_object.size() > FUSION_COMPRESS_OBJECT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_COMPRESS_OBJECT_MAX_NUM; i0++) {
        convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.fusion_object.size(); i0++) {
        convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
      }
    }
  }
  //
}

