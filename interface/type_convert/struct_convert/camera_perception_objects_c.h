#pragma once

#include "base_convert.h"
#include "c/camera_perception_objects_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraPerception2DBoundingBox &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.top_left_x, ros_v.top_left_x, type);
  convert(struct_v.top_left_y, ros_v.top_left_y, type);
  convert(struct_v.bottom_right_x, ros_v.bottom_right_x, type);
  convert(struct_v.bottom_right_y, ros_v.bottom_right_y, type);
  convert(struct_v.mosaic_top_left_x, ros_v.mosaic_top_left_x, type);
  convert(struct_v.mosaic_top_left_y, ros_v.mosaic_top_left_y, type);
  convert(struct_v.mosaic_bottom_right_x, ros_v.mosaic_bottom_right_x, type);
  convert(struct_v.mosaic_bottom_right_y, ros_v.mosaic_bottom_right_y, type);
}

template <typename T2>
void convert(iflyauto::CameraPerception2DBoundingBoxes &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.camera_type, ros_v.camera_type, type);
  convert(struct_v.camera_width, ros_v.camera_width, type);
  convert(struct_v.camera_height, ros_v.camera_height, type);
  convert(struct_v.bounding_box_2d_size, ros_v.bounding_box_2d_size, type);
  for (size_t i0 = 0; i0 < ros_v.boundingbox_2d.size(); i0++) {
	  convert(struct_v.boundingbox_2d[i0], ros_v.boundingbox_2d[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::CameraPerceptionAllView2DBoundingBoxes &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.view_size, ros_v.view_size, type);
  for (size_t i0 = 0; i0 < ros_v.boundingboxes_2d.size(); i0++) {
	  convert(struct_v.boundingboxes_2d[i0], ros_v.boundingboxes_2d[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::CameraPerceptionAdditional &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.sensor_type, ros_v.sensor_type, type);
  convert(struct_v.bounding_box_points_size, ros_v.bounding_box_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.bounding_box_points_size >= 0 && struct_v.bounding_box_points_size <= CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM) {
      ros_v.bounding_box_points.resize(struct_v.bounding_box_points_size);
    } else {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] bounding_box_points_size=" << struct_v.bounding_box_points_size 
                << " not in range CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM 
                << std::endl;
      ros_v.bounding_box_points_size = CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM;
      ros_v.bounding_box_points.resize(CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.bounding_box_points.size(); i0++) {
      convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.bounding_box_points_size > CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM || ros_v.bounding_box_points_size < 0 || ros_v.bounding_box_points.size() > CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM) {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] bounding_box_points_size=" << ros_v.bounding_box_points_size 
                << " ros_v.bounding_box_points.size()=" << ros_v.bounding_box_points.size()
                << " not in range CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.bounding_box_points.size() > CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM; i0++) {
        convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.bounding_box_points.size(); i0++) {
        convert(struct_v.bounding_box_points[i0], ros_v.bounding_box_points[i0], type);
      }
    }
  }
  //
  convert(struct_v.contour_points_size, ros_v.contour_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.contour_points_size >= 0 && struct_v.contour_points_size <= CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM) {
      ros_v.contour_points.resize(struct_v.contour_points_size);
    } else {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] contour_points_size=" << struct_v.contour_points_size 
                << " not in range CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM 
                << std::endl;
      ros_v.contour_points_size = CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM;
      ros_v.contour_points.resize(CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.contour_points.size(); i1++) {
      convert(struct_v.contour_points[i1], ros_v.contour_points[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.contour_points_size > CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM || ros_v.contour_points_size < 0 || ros_v.contour_points.size() > CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM) {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] contour_points_size=" << ros_v.contour_points_size 
                << " ros_v.contour_points.size()=" << ros_v.contour_points.size()
                << " not in range CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.contour_points.size() > CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM; i1++) {
        convert(struct_v.contour_points[i1], ros_v.contour_points[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.contour_points.size(); i1++) {
        convert(struct_v.contour_points[i1], ros_v.contour_points[i1], type);
      }
    }
  }
  //
  convert(struct_v.life_time, ros_v.life_time, type);
  convert(struct_v.age, ros_v.age, type);
  convert(struct_v.conf, ros_v.conf, type);
  convert(struct_v.orientation_angle_conf, ros_v.orientation_angle_conf, type);
  convert(struct_v.vel_conf, ros_v.vel_conf, type);
  convert(struct_v.accel_conf, ros_v.accel_conf, type);
  convert(struct_v.is_fusion, ros_v.is_fusion, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.common_info, ros_v.common_info, type);
  convert(struct_v.additional_info, ros_v.additional_info, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionObjectsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.camera_perception_objects_size, ros_v.camera_perception_objects_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.camera_perception_objects_size >= 0 && struct_v.camera_perception_objects_size <= CAMERA_PERCEPTION_OBJECT_MAX_NUM) {
      ros_v.camera_perception_objects.resize(struct_v.camera_perception_objects_size);
    } else {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] camera_perception_objects_size=" << struct_v.camera_perception_objects_size 
                << " not in range CAMERA_PERCEPTION_OBJECT_MAX_NUM=" << CAMERA_PERCEPTION_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.camera_perception_objects_size = CAMERA_PERCEPTION_OBJECT_MAX_NUM;
      ros_v.camera_perception_objects.resize(CAMERA_PERCEPTION_OBJECT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.camera_perception_objects.size(); i0++) {
      convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.camera_perception_objects_size > CAMERA_PERCEPTION_OBJECT_MAX_NUM || ros_v.camera_perception_objects_size < 0 || ros_v.camera_perception_objects.size() > CAMERA_PERCEPTION_OBJECT_MAX_NUM) {
      std::cout << "convert/camera_perception_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] camera_perception_objects_size=" << ros_v.camera_perception_objects_size 
                << " ros_v.camera_perception_objects.size()=" << ros_v.camera_perception_objects.size()
                << " not in range CAMERA_PERCEPTION_OBJECT_MAX_NUM=" << CAMERA_PERCEPTION_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.camera_perception_objects.size() > CAMERA_PERCEPTION_OBJECT_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_OBJECT_MAX_NUM; i0++) {
        convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.camera_perception_objects.size(); i0++) {
        convert(struct_v.camera_perception_objects[i0], ros_v.camera_perception_objects[i0], type);
      }
    }
  }
  //
  convert(struct_v.camera_perception_input_timestamp, ros_v.camera_perception_input_timestamp, type);
}

