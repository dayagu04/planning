#pragma once

#include "base_convert.h"
#include "c/camera_perception_occupancy_grid_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraPerceptionOccGridMeta &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.grid_resolution_x, ros_v.grid_resolution_x, type);
  convert(struct_v.grid_resolution_y, ros_v.grid_resolution_y, type);
  convert(struct_v.grid_resolution_z, ros_v.grid_resolution_z, type);
  convert(struct_v.grid_size_positive_x, ros_v.grid_size_positive_x, type);
  convert(struct_v.grid_size_negative_x, ros_v.grid_size_negative_x, type);
  convert(struct_v.grid_size_positive_y, ros_v.grid_size_positive_y, type);
  convert(struct_v.grid_size_negative_y, ros_v.grid_size_negative_y, type);
  convert(struct_v.grid_size_positive_z, ros_v.grid_size_positive_z, type);
  convert(struct_v.grid_size_negative_z, ros_v.grid_size_negative_z, type);
}

template <typename T2>
void convert(iflyauto::CameraPerceptionOccGridInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  for (size_t i0 = 0; i0 < ros_v.isp_ego_pose.size(); i0++) {
	  convert(struct_v.isp_ego_pose[i0], ros_v.isp_ego_pose[i0], type);
  }
  convert(struct_v.occ_grid_meta, ros_v.occ_grid_meta, type);
  convert(struct_v.occ_grid_num, ros_v.occ_grid_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.occ_grid_num >= 0 && struct_v.occ_grid_num <= CAMERA_PERCEPTION_OCC_GRID_MAX_NUM) {
      ros_v.occ_grid.resize(struct_v.occ_grid_num);
    } else {
      std::cout << "convert/camera_perception_occupancy_grid_c.h:" << __LINE__ 
                << " [convert][TO_ROS] occ_grid_num=" << struct_v.occ_grid_num 
                << " not in range CAMERA_PERCEPTION_OCC_GRID_MAX_NUM=" << CAMERA_PERCEPTION_OCC_GRID_MAX_NUM 
                << std::endl;
      ros_v.occ_grid_num = CAMERA_PERCEPTION_OCC_GRID_MAX_NUM;
      ros_v.occ_grid.resize(CAMERA_PERCEPTION_OCC_GRID_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.occ_grid.size(); i1++) {
      convert(struct_v.occ_grid[i1], ros_v.occ_grid[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.occ_grid_num > CAMERA_PERCEPTION_OCC_GRID_MAX_NUM || ros_v.occ_grid_num < 0 || ros_v.occ_grid.size() > CAMERA_PERCEPTION_OCC_GRID_MAX_NUM) {
      std::cout << "convert/camera_perception_occupancy_grid_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] occ_grid_num=" << ros_v.occ_grid_num 
                << " ros_v.occ_grid.size()=" << ros_v.occ_grid.size()
                << " not in range CAMERA_PERCEPTION_OCC_GRID_MAX_NUM=" << CAMERA_PERCEPTION_OCC_GRID_MAX_NUM 
                << std::endl;
    }
    if (ros_v.occ_grid.size() > CAMERA_PERCEPTION_OCC_GRID_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_OCC_GRID_MAX_NUM; i1++) {
        convert(struct_v.occ_grid[i1], ros_v.occ_grid[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.occ_grid.size(); i1++) {
        convert(struct_v.occ_grid[i1], ros_v.occ_grid[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::CameraPerceptionDrivableSpaceGridInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  for (size_t i0 = 0; i0 < ros_v.isp_ego_pose.size(); i0++) {
	  convert(struct_v.isp_ego_pose[i0], ros_v.isp_ego_pose[i0], type);
  }
  convert(struct_v.drivable_space_grid_meta, ros_v.drivable_space_grid_meta, type);
  convert(struct_v.drivable_space_grid_num, ros_v.drivable_space_grid_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.drivable_space_grid_num >= 0 && struct_v.drivable_space_grid_num <= CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM) {
      ros_v.drivable_space_grid.resize(struct_v.drivable_space_grid_num);
    } else {
      std::cout << "convert/camera_perception_occupancy_grid_c.h:" << __LINE__ 
                << " [convert][TO_ROS] drivable_space_grid_num=" << struct_v.drivable_space_grid_num 
                << " not in range CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM=" << CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM 
                << std::endl;
      ros_v.drivable_space_grid_num = CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM;
      ros_v.drivable_space_grid.resize(CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.drivable_space_grid.size(); i1++) {
      convert(struct_v.drivable_space_grid[i1], ros_v.drivable_space_grid[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.drivable_space_grid_num > CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM || ros_v.drivable_space_grid_num < 0 || ros_v.drivable_space_grid.size() > CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM) {
      std::cout << "convert/camera_perception_occupancy_grid_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] drivable_space_grid_num=" << ros_v.drivable_space_grid_num 
                << " ros_v.drivable_space_grid.size()=" << ros_v.drivable_space_grid.size()
                << " not in range CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM=" << CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM 
                << std::endl;
    }
    if (ros_v.drivable_space_grid.size() > CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM; i1++) {
        convert(struct_v.drivable_space_grid[i1], ros_v.drivable_space_grid[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.drivable_space_grid.size(); i1++) {
        convert(struct_v.drivable_space_grid[i1], ros_v.drivable_space_grid[i1], type);
      }
    }
  }
  //
}

