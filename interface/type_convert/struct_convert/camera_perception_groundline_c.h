#pragma once

#include "base_convert.h"
#include "c/camera_perception_groundline_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::GroundLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.camera_source, ros_v.camera_source, type);
  convert(struct_v.camera_model, ros_v.camera_model, type);
  convert(struct_v.semantic_type, ros_v.semantic_type, type);
  convert(struct_v.point_type, ros_v.point_type, type);
  convert(struct_v.visable_seg_num, ros_v.visable_seg_num, type);
  convert(struct_v.points_2d_size, ros_v.points_2d_size, type);
  convert(struct_v.points_3d_size, ros_v.points_3d_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.points_2d_size >= 0 && struct_v.points_2d_size <= CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      ros_v.points_2d.resize(struct_v.points_2d_size);
    } else {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << " [convert][TO_ROS] points_2d_size=" << struct_v.points_2d_size 
                << " not in range CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM 
                << std::endl;
      ros_v.points_2d_size = CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM;
      ros_v.points_2d.resize(CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.points_2d.size(); i0++) {
      convert(struct_v.points_2d[i0], ros_v.points_2d[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.points_2d_size > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM || ros_v.points_2d_size < 0 || ros_v.points_2d.size() > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] points_2d_size=" << ros_v.points_2d_size 
                << " ros_v.points_2d.size()=" << ros_v.points_2d.size()
                << " not in range CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.points_2d.size() > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM; i0++) {
        convert(struct_v.points_2d[i0], ros_v.points_2d[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.points_2d.size(); i0++) {
        convert(struct_v.points_2d[i0], ros_v.points_2d[i0], type);
      }
    }
  }
  //
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.points_3d_size >= 0 && struct_v.points_3d_size <= CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      ros_v.points_3d.resize(struct_v.points_3d_size);
    } else {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << " [convert][TO_ROS] points_3d_size=" << struct_v.points_3d_size 
                << " not in range CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM 
                << std::endl;
      ros_v.points_3d_size = CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM;
      ros_v.points_3d.resize(CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.points_3d.size(); i1++) {
      convert(struct_v.points_3d[i1], ros_v.points_3d[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.points_3d_size > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM || ros_v.points_3d_size < 0 || ros_v.points_3d.size() > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] points_3d_size=" << ros_v.points_3d_size 
                << " ros_v.points_3d.size()=" << ros_v.points_3d.size()
                << " not in range CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.points_3d.size() > CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM; i1++) {
        convert(struct_v.points_3d[i1], ros_v.points_3d[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.points_3d.size(); i1++) {
        convert(struct_v.points_3d[i1], ros_v.points_3d[i1], type);
      }
    }
  }
  //
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::GroundLinePerceptionInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.ground_lines_size, ros_v.ground_lines_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ground_lines_size >= 0 && struct_v.ground_lines_size <= CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      ros_v.ground_lines.resize(struct_v.ground_lines_size);
    } else {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ground_lines_size=" << struct_v.ground_lines_size 
                << " not in range CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM 
                << std::endl;
      ros_v.ground_lines_size = CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM;
      ros_v.ground_lines.resize(CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.ground_lines.size(); i0++) {
      convert(struct_v.ground_lines[i0], ros_v.ground_lines[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ground_lines_size > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM || ros_v.ground_lines_size < 0 || ros_v.ground_lines.size() > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      std::cout << "convert/camera_perception_groundline_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ground_lines_size=" << ros_v.ground_lines_size 
                << " ros_v.ground_lines.size()=" << ros_v.ground_lines.size()
                << " not in range CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM 
                << std::endl;
    }
    if (ros_v.ground_lines.size() > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM; i0++) {
        convert(struct_v.ground_lines[i0], ros_v.ground_lines[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ground_lines.size(); i0++) {
        convert(struct_v.ground_lines[i0], ros_v.ground_lines[i0], type);
      }
    }
  }
  //
  for (size_t i1 = 0; i1 < ros_v.reserved_infos.size(); i1++) {
	  convert(struct_v.reserved_infos[i1], ros_v.reserved_infos[i1], type);
  }
}

