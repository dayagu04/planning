#pragma once

#include "base_convert.h"
#include "c/camera_perception_deceler_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::Deceler &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  for (size_t i0 = 0; i0 < ros_v.deceler_points.size(); i0++) {
	  convert(struct_v.deceler_points[i0], ros_v.deceler_points[i0], type);
  }
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.life_time, ros_v.life_time, type);
}

template <typename T2>
void convert(iflyauto::DecelerPerceptionInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.decelers_size, ros_v.decelers_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.decelers_size >= 0 && struct_v.decelers_size <= CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      ros_v.decelers.resize(struct_v.decelers_size);
    } else {
      std::cout << "convert/camera_perception_deceler_c.h:" << __LINE__ 
                << " [convert][TO_ROS] decelers_size=" << struct_v.decelers_size 
                << " not in range CAMERA_PERCEPTION_DECELERS_MAX_NUM=" << CAMERA_PERCEPTION_DECELERS_MAX_NUM 
                << std::endl;
      ros_v.decelers_size = CAMERA_PERCEPTION_DECELERS_MAX_NUM;
      ros_v.decelers.resize(CAMERA_PERCEPTION_DECELERS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.decelers.size(); i0++) {
      convert(struct_v.decelers[i0], ros_v.decelers[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.decelers_size > CAMERA_PERCEPTION_DECELERS_MAX_NUM || ros_v.decelers_size < 0 || ros_v.decelers.size() > CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      std::cout << "convert/camera_perception_deceler_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] decelers_size=" << ros_v.decelers_size 
                << " ros_v.decelers.size()=" << ros_v.decelers.size()
                << " not in range CAMERA_PERCEPTION_DECELERS_MAX_NUM=" << CAMERA_PERCEPTION_DECELERS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.decelers.size() > CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_DECELERS_MAX_NUM; i0++) {
        convert(struct_v.decelers[i0], ros_v.decelers[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.decelers.size(); i0++) {
        convert(struct_v.decelers[i0], ros_v.decelers[i0], type);
      }
    }
  }
  //
}

