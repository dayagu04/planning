#pragma once

#include "base_convert.h"
#include "c/camera_perception_feature_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/camera_perception_objects_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraPerceptionFeatureInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.index, ros_v.index, type);
  convert(struct_v.bev_feature_dim, ros_v.bev_feature_dim, type);
  convert(struct_v.bev_feature_num, ros_v.bev_feature_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.bev_feature_num >= 0 && struct_v.bev_feature_num <= CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM) {
      ros_v.bev_feature.resize(struct_v.bev_feature_num);
    } else {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << " [convert][TO_ROS] bev_feature_num=" << struct_v.bev_feature_num 
                << " not in range CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM 
                << std::endl;
      ros_v.bev_feature_num = CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM;
      ros_v.bev_feature.resize(CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.bev_feature.size(); i0++) {
      convert(struct_v.bev_feature[i0], ros_v.bev_feature[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.bev_feature_num > CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM || ros_v.bev_feature_num < 0 || ros_v.bev_feature.size() > CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM) {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] bev_feature_num=" << ros_v.bev_feature_num 
                << " ros_v.bev_feature.size()=" << ros_v.bev_feature.size()
                << " not in range CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.bev_feature.size() > CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM; i0++) {
        convert(struct_v.bev_feature[i0], ros_v.bev_feature[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.bev_feature.size(); i0++) {
        convert(struct_v.bev_feature[i0], ros_v.bev_feature[i0], type);
      }
    }
  }
  //
  convert(struct_v.img_feature_dim, ros_v.img_feature_dim, type);
  convert(struct_v.img_feature_num, ros_v.img_feature_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.img_feature_num >= 0 && struct_v.img_feature_num <= CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM) {
      ros_v.img_feature.resize(struct_v.img_feature_num);
    } else {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << " [convert][TO_ROS] img_feature_num=" << struct_v.img_feature_num 
                << " not in range CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM 
                << std::endl;
      ros_v.img_feature_num = CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM;
      ros_v.img_feature.resize(CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.img_feature.size(); i1++) {
      convert(struct_v.img_feature[i1], ros_v.img_feature[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.img_feature_num > CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM || ros_v.img_feature_num < 0 || ros_v.img_feature.size() > CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM) {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] img_feature_num=" << ros_v.img_feature_num 
                << " ros_v.img_feature.size()=" << ros_v.img_feature.size()
                << " not in range CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.img_feature.size() > CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM; i1++) {
        convert(struct_v.img_feature[i1], ros_v.img_feature[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.img_feature.size(); i1++) {
        convert(struct_v.img_feature[i1], ros_v.img_feature[i1], type);
      }
    }
  }
  //
  convert(struct_v.reserved, ros_v.reserved, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionLaneFeatureInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.index, ros_v.index, type);
  convert(struct_v.img_feature_dim, ros_v.img_feature_dim, type);
  convert(struct_v.img_feature_num, ros_v.img_feature_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.img_feature_num >= 0 && struct_v.img_feature_num <= CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM) {
      ros_v.img_feature.resize(struct_v.img_feature_num);
    } else {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << " [convert][TO_ROS] img_feature_num=" << struct_v.img_feature_num 
                << " not in range CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM 
                << std::endl;
      ros_v.img_feature_num = CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM;
      ros_v.img_feature.resize(CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.img_feature.size(); i0++) {
      convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.img_feature_num > CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM || ros_v.img_feature_num < 0 || ros_v.img_feature.size() > CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM) {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] img_feature_num=" << ros_v.img_feature_num 
                << " ros_v.img_feature.size()=" << ros_v.img_feature.size()
                << " not in range CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.img_feature.size() > CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM; i0++) {
        convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.img_feature.size(); i0++) {
        convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
      }
    }
  }
  //
  convert(struct_v.reserved, ros_v.reserved, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionObstacleFeatureInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.index, ros_v.index, type);
  convert(struct_v.img_feature_dim, ros_v.img_feature_dim, type);
  convert(struct_v.img_feature_num, ros_v.img_feature_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.img_feature_num >= 0 && struct_v.img_feature_num <= CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM) {
      ros_v.img_feature.resize(struct_v.img_feature_num);
    } else {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << " [convert][TO_ROS] img_feature_num=" << struct_v.img_feature_num 
                << " not in range CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM 
                << std::endl;
      ros_v.img_feature_num = CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM;
      ros_v.img_feature.resize(CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.img_feature.size(); i0++) {
      convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.img_feature_num > CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM || ros_v.img_feature_num < 0 || ros_v.img_feature.size() > CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM) {
      std::cout << "convert/camera_perception_feature_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] img_feature_num=" << ros_v.img_feature_num 
                << " ros_v.img_feature.size()=" << ros_v.img_feature.size()
                << " not in range CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM=" << CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.img_feature.size() > CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM; i0++) {
        convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.img_feature.size(); i0++) {
        convert(struct_v.img_feature[i0], ros_v.img_feature[i0], type);
      }
    }
  }
  //
  convert(struct_v.reserved, ros_v.reserved, type);
}

