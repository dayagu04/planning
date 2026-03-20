#pragma once

#include "base_convert.h"
#include "c/camera_perception_tsr_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::TrafficSignBoundingbox &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.width, ros_v.width, type);
  convert(struct_v.height, ros_v.height, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionSuppSign &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.supp_sign_type, ros_v.supp_sign_type, type);
  convert(struct_v.supp_sign_x, ros_v.supp_sign_x, type);
  convert(struct_v.supp_sign_y, ros_v.supp_sign_y, type);
  convert(struct_v.supp_sign_z, ros_v.supp_sign_z, type);
  convert(struct_v.speed_limit, ros_v.speed_limit, type);
  convert(struct_v.bbox, ros_v.bbox, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionTrafficLight &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.traffic_light_type, ros_v.traffic_light_type, type);
  convert(struct_v.traffic_light_color, ros_v.traffic_light_color, type);
  convert(struct_v.traffic_light_arrange_type, ros_v.traffic_light_arrange_type, type);
  convert(struct_v.traffic_light_x, ros_v.traffic_light_x, type);
  convert(struct_v.traffic_light_y, ros_v.traffic_light_y, type);
  convert(struct_v.traffic_light_z, ros_v.traffic_light_z, type);
  convert(struct_v.bbox, ros_v.bbox, type);
  convert(struct_v.countdown_number, ros_v.countdown_number, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionTrafficStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.go_left, ros_v.go_left, type);
  convert(struct_v.go_straight, ros_v.go_straight, type);
  convert(struct_v.go_right, ros_v.go_right, type);
  convert(struct_v.go_uturn, ros_v.go_uturn, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionTsrInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.supp_signs_size, ros_v.supp_signs_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.supp_signs_size >= 0 && struct_v.supp_signs_size <= CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      ros_v.supp_signs.resize(struct_v.supp_signs_size);
    } else {
      std::cout << "convert/camera_perception_tsr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] supp_signs_size=" << struct_v.supp_signs_size 
                << " not in range CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM=" << CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM 
                << std::endl;
      ros_v.supp_signs_size = CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM;
      ros_v.supp_signs.resize(CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.supp_signs.size(); i0++) {
      convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.supp_signs_size > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM || ros_v.supp_signs_size < 0 || ros_v.supp_signs.size() > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      std::cout << "convert/camera_perception_tsr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] supp_signs_size=" << ros_v.supp_signs_size 
                << " ros_v.supp_signs.size()=" << ros_v.supp_signs.size()
                << " not in range CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM=" << CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.supp_signs.size() > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM; i0++) {
        convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.supp_signs.size(); i0++) {
        convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
      }
    }
  }
  //
  convert(struct_v.traffic_lights_size, ros_v.traffic_lights_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.traffic_lights_size >= 0 && struct_v.traffic_lights_size <= CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      ros_v.traffic_lights.resize(struct_v.traffic_lights_size);
    } else {
      std::cout << "convert/camera_perception_tsr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] traffic_lights_size=" << struct_v.traffic_lights_size 
                << " not in range CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM=" << CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM 
                << std::endl;
      ros_v.traffic_lights_size = CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM;
      ros_v.traffic_lights.resize(CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.traffic_lights.size(); i1++) {
      convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.traffic_lights_size > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM || ros_v.traffic_lights_size < 0 || ros_v.traffic_lights.size() > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      std::cout << "convert/camera_perception_tsr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] traffic_lights_size=" << ros_v.traffic_lights_size 
                << " ros_v.traffic_lights.size()=" << ros_v.traffic_lights.size()
                << " not in range CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM=" << CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.traffic_lights.size() > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM; i1++) {
        convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.traffic_lights.size(); i1++) {
        convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
      }
    }
  }
  //
  convert(struct_v.traffic_status, ros_v.traffic_status, type);
}

