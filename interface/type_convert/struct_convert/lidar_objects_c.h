#pragma once

#include "base_convert.h"
#include "c/lidar_objects_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::LidarAdditional &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.track_age, ros_v.track_age, type);
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::LidarObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.common_info, ros_v.common_info, type);
  convert(struct_v.additional_info, ros_v.additional_info, type);
}

template <typename T2>
void convert(iflyauto::LidarObjectsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.lidar_object_size, ros_v.lidar_object_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lidar_object_size >= 0 && struct_v.lidar_object_size <= LIDAR_OBJECT_MAX_NUM) {
      ros_v.lidar_object.resize(struct_v.lidar_object_size);
    } else {
      std::cout << "convert/lidar_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lidar_object_size=" << struct_v.lidar_object_size 
                << " not in range LIDAR_OBJECT_MAX_NUM=" << LIDAR_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.lidar_object_size = LIDAR_OBJECT_MAX_NUM;
      ros_v.lidar_object.resize(LIDAR_OBJECT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lidar_object.size(); i0++) {
      convert(struct_v.lidar_object[i0], ros_v.lidar_object[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lidar_object_size > LIDAR_OBJECT_MAX_NUM || ros_v.lidar_object_size < 0 || ros_v.lidar_object.size() > LIDAR_OBJECT_MAX_NUM) {
      std::cout << "convert/lidar_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lidar_object_size=" << ros_v.lidar_object_size 
                << " ros_v.lidar_object.size()=" << ros_v.lidar_object.size()
                << " not in range LIDAR_OBJECT_MAX_NUM=" << LIDAR_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lidar_object.size() > LIDAR_OBJECT_MAX_NUM) {
      for (size_t i0 = 0; i0 < LIDAR_OBJECT_MAX_NUM; i0++) {
        convert(struct_v.lidar_object[i0], ros_v.lidar_object[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lidar_object.size(); i0++) {
        convert(struct_v.lidar_object[i0], ros_v.lidar_object[i0], type);
      }
    }
  }
  //
}

