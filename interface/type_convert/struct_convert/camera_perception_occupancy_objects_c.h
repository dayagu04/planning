#pragma once

#include "base_convert.h"
#include "c/camera_perception_occupancy_objects_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraPerceptionOccObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.visable_seg_num, ros_v.visable_seg_num, type);
  convert(struct_v.contour_points_size, ros_v.contour_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.contour_points_size >= 0 && struct_v.contour_points_size <= CAMERA_PERCEPTION_OCC_POINT_MAX_NUM) {
      ros_v.contour_points.resize(struct_v.contour_points_size);
    } else {
      std::cout << "convert/camera_perception_occupancy_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] contour_points_size=" << struct_v.contour_points_size 
                << " not in range CAMERA_PERCEPTION_OCC_POINT_MAX_NUM=" << CAMERA_PERCEPTION_OCC_POINT_MAX_NUM 
                << std::endl;
      ros_v.contour_points_size = CAMERA_PERCEPTION_OCC_POINT_MAX_NUM;
      ros_v.contour_points.resize(CAMERA_PERCEPTION_OCC_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.contour_points.size(); i0++) {
      convert(struct_v.contour_points[i0], ros_v.contour_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.contour_points_size > CAMERA_PERCEPTION_OCC_POINT_MAX_NUM || ros_v.contour_points_size < 0 || ros_v.contour_points.size() > CAMERA_PERCEPTION_OCC_POINT_MAX_NUM) {
      std::cout << "convert/camera_perception_occupancy_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] contour_points_size=" << ros_v.contour_points_size 
                << " ros_v.contour_points.size()=" << ros_v.contour_points.size()
                << " not in range CAMERA_PERCEPTION_OCC_POINT_MAX_NUM=" << CAMERA_PERCEPTION_OCC_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.contour_points.size() > CAMERA_PERCEPTION_OCC_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_OCC_POINT_MAX_NUM; i0++) {
        convert(struct_v.contour_points[i0], ros_v.contour_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.contour_points.size(); i0++) {
        convert(struct_v.contour_points[i0], ros_v.contour_points[i0], type);
      }
    }
  }
  //
  convert(struct_v.life_time, ros_v.life_time, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionOccObjectsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.camera_perception_objects_size, ros_v.camera_perception_objects_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.camera_perception_objects_size >= 0 && struct_v.camera_perception_objects_size <= CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM) {
      ros_v.camera_perception_objects.resize(struct_v.camera_perception_objects_size);
    } else {
      std::cout << "convert/camera_perception_occupancy_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] camera_perception_objects_size=" << struct_v.camera_perception_objects_size 
                << " not in range CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM=" << CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.camera_perception_objects_size = CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM;
      ros_v.camera_perception_objects.resize(CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.camera_perception_objects.size(); i0++) {
      convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.camera_perception_objects_size > CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM || ros_v.camera_perception_objects_size < 0 || ros_v.camera_perception_objects.size() > CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM) {
      std::cout << "convert/camera_perception_occupancy_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] camera_perception_objects_size=" << ros_v.camera_perception_objects_size 
                << " ros_v.camera_perception_objects.size()=" << ros_v.camera_perception_objects.size()
                << " not in range CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM=" << CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.camera_perception_objects.size() > CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM; i0++) {
        convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.camera_perception_objects.size(); i0++) {
        convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
      }
    }
  }
  //
}

