#pragma once

#include "base_convert.h"
#include "c/fusion_objects_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FusionObjectsAdditional &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.motion_pattern_current, ros_v.motion_pattern_current, type);
  convert(struct_v.motion_pattern_history, ros_v.motion_pattern_history, type);
  convert(struct_v.fusion_source, ros_v.fusion_source, type);
  convert(struct_v.track_id, ros_v.track_id, type);
  convert(struct_v.track_age, ros_v.track_age, type);
  convert(struct_v.confidence, ros_v.confidence, type);
  convert(struct_v.track_status, ros_v.track_status, type);
  convert(struct_v.bounding_box_points_size, ros_v.bounding_box_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.bounding_box_points_size >= 0 && struct_v.bounding_box_points_size <= FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM) {
      ros_v.bounding_box_points.resize(struct_v.bounding_box_points_size);
    } else {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] bounding_box_points_size=" << struct_v.bounding_box_points_size 
                << " not in range FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM=" << FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM 
                << std::endl;
      ros_v.bounding_box_points_size = FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM;
      ros_v.bounding_box_points.resize(FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.bounding_box_points.size(); i0++) {
      convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.bounding_box_points_size > FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM || ros_v.bounding_box_points_size < 0 || ros_v.bounding_box_points.size() > FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM) {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] bounding_box_points_size=" << ros_v.bounding_box_points_size 
                << " ros_v.bounding_box_points.size()=" << ros_v.bounding_box_points.size()
                << " not in range FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM=" << FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM 
                << std::endl;
    }
    if (ros_v.bounding_box_points.size() > FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM; i0++) {
        convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.bounding_box_points.size(); i0++) {
        convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
      }
    }
  }
  //
  convert(struct_v.polygon_points_size, ros_v.polygon_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.polygon_points_size >= 0 && struct_v.polygon_points_size <= FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM) {
      ros_v.polygon_points.resize(struct_v.polygon_points_size);
    } else {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] polygon_points_size=" << struct_v.polygon_points_size 
                << " not in range FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM=" << FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM 
                << std::endl;
      ros_v.polygon_points_size = FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM;
      ros_v.polygon_points.resize(FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.polygon_points.size(); i1++) {
      convert(struct_v.polygon_points[i1], ros_v.polygon_points[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.polygon_points_size > FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM || ros_v.polygon_points_size < 0 || ros_v.polygon_points.size() > FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM) {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] polygon_points_size=" << ros_v.polygon_points_size 
                << " ros_v.polygon_points.size()=" << ros_v.polygon_points.size()
                << " not in range FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM=" << FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM 
                << std::endl;
    }
    if (ros_v.polygon_points.size() > FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM; i1++) {
        convert(struct_v.polygon_points[i1], ros_v.polygon_points[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.polygon_points.size(); i1++) {
        convert(struct_v.polygon_points[i1], ros_v.polygon_points[i1], type);
      }
    }
  }
  //
  convert(struct_v.relative_speed_angle, ros_v.relative_speed_angle, type);
  convert(struct_v.sensor_source_size, ros_v.sensor_source_size, type);
  for (size_t i2 = 0; i2 < ros_v.sensor_source_id.size(); i2++) {
	  convert(struct_v.sensor_source_id[i2], ros_v.sensor_source_id[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::FusionObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.common_info, ros_v.common_info, type);
  convert(struct_v.additional_info, ros_v.additional_info, type);
}

template <typename T2>
void convert(iflyauto::FusionObjectsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.fusion_object_size, ros_v.fusion_object_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.fusion_object_size >= 0 && struct_v.fusion_object_size <= FUSION_OBJECT_MAX_NUM) {
      ros_v.fusion_object.resize(struct_v.fusion_object_size);
    } else {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] fusion_object_size=" << struct_v.fusion_object_size 
                << " not in range FUSION_OBJECT_MAX_NUM=" << FUSION_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.fusion_object_size = FUSION_OBJECT_MAX_NUM;
      ros_v.fusion_object.resize(FUSION_OBJECT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.fusion_object.size(); i0++) {
      convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.fusion_object_size > FUSION_OBJECT_MAX_NUM || ros_v.fusion_object_size < 0 || ros_v.fusion_object.size() > FUSION_OBJECT_MAX_NUM) {
      std::cout << "convert/fusion_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] fusion_object_size=" << ros_v.fusion_object_size 
                << " ros_v.fusion_object.size()=" << ros_v.fusion_object.size()
                << " not in range FUSION_OBJECT_MAX_NUM=" << FUSION_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.fusion_object.size() > FUSION_OBJECT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_OBJECT_MAX_NUM; i0++) {
        convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.fusion_object.size(); i0++) {
        convert(struct_v.fusion_object[i0], ros_v.fusion_object[i0], type);
      }
    }
  }
  //
  convert(struct_v.local_point_valid, ros_v.local_point_valid, type);
  convert(struct_v.perception_mode, ros_v.perception_mode, type);
}

