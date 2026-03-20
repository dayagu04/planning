#pragma once

#include "base_convert.h"
#include "c/fusion_groundline_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FusionGroundLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.resource_type, ros_v.resource_type, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.groundline_point_size, ros_v.groundline_point_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.groundline_point_size >= 0 && struct_v.groundline_point_size <= FUSION_GROUNDLINE_POINT_MAX_NUM) {
      ros_v.groundline_point.resize(struct_v.groundline_point_size);
    } else {
      std::cout << "convert/fusion_groundline_c.h:" << __LINE__ 
                << " [convert][TO_ROS] groundline_point_size=" << struct_v.groundline_point_size 
                << " not in range FUSION_GROUNDLINE_POINT_MAX_NUM=" << FUSION_GROUNDLINE_POINT_MAX_NUM 
                << std::endl;
      ros_v.groundline_point_size = FUSION_GROUNDLINE_POINT_MAX_NUM;
      ros_v.groundline_point.resize(FUSION_GROUNDLINE_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.groundline_point.size(); i0++) {
      convert(struct_v.groundline_point[i0], ros_v.groundline_point[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.groundline_point_size > FUSION_GROUNDLINE_POINT_MAX_NUM || ros_v.groundline_point_size < 0 || ros_v.groundline_point.size() > FUSION_GROUNDLINE_POINT_MAX_NUM) {
      std::cout << "convert/fusion_groundline_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] groundline_point_size=" << ros_v.groundline_point_size 
                << " ros_v.groundline_point.size()=" << ros_v.groundline_point.size()
                << " not in range FUSION_GROUNDLINE_POINT_MAX_NUM=" << FUSION_GROUNDLINE_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.groundline_point.size() > FUSION_GROUNDLINE_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_GROUNDLINE_POINT_MAX_NUM; i0++) {
        convert(struct_v.groundline_point[i0], ros_v.groundline_point[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.groundline_point.size(); i0++) {
        convert(struct_v.groundline_point[i0], ros_v.groundline_point[i0], type);
      }
    }
  }
  //
  convert(struct_v.reserved, ros_v.reserved, type);
}

template <typename T2>
void convert(iflyauto::FusionGroundLineInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.groundline_size, ros_v.groundline_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.groundline_size >= 0 && struct_v.groundline_size <= FUSION_GROUNDLINE_MAX_NUM) {
      ros_v.groundline.resize(struct_v.groundline_size);
    } else {
      std::cout << "convert/fusion_groundline_c.h:" << __LINE__ 
                << " [convert][TO_ROS] groundline_size=" << struct_v.groundline_size 
                << " not in range FUSION_GROUNDLINE_MAX_NUM=" << FUSION_GROUNDLINE_MAX_NUM 
                << std::endl;
      ros_v.groundline_size = FUSION_GROUNDLINE_MAX_NUM;
      ros_v.groundline.resize(FUSION_GROUNDLINE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.groundline.size(); i0++) {
      convert(struct_v.groundline[i0], ros_v.groundline[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.groundline_size > FUSION_GROUNDLINE_MAX_NUM || ros_v.groundline_size < 0 || ros_v.groundline.size() > FUSION_GROUNDLINE_MAX_NUM) {
      std::cout << "convert/fusion_groundline_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] groundline_size=" << ros_v.groundline_size 
                << " ros_v.groundline.size()=" << ros_v.groundline.size()
                << " not in range FUSION_GROUNDLINE_MAX_NUM=" << FUSION_GROUNDLINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.groundline.size() > FUSION_GROUNDLINE_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_GROUNDLINE_MAX_NUM; i0++) {
        convert(struct_v.groundline[i0], ros_v.groundline[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.groundline.size(); i0++) {
        convert(struct_v.groundline[i0], ros_v.groundline[i0], type);
      }
    }
  }
  //
}

