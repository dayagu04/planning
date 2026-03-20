#pragma once

#include "base_convert.h"
#include "c/radar_perception_objects_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::RadarPerceptionObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.relative_position, ros_v.relative_position, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_acceleration, ros_v.relative_acceleration, type);
  convert(struct_v.relative_heading_angle, ros_v.relative_heading_angle, type);
  convert(struct_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
  convert(struct_v.age, ros_v.age, type);
  convert(struct_v.conf, ros_v.conf, type);
  convert(struct_v.orientation_angle_conf, ros_v.orientation_angle_conf, type);
  convert(struct_v.vel_conf, ros_v.vel_conf, type);
  convert(struct_v.accel_conf, ros_v.accel_conf, type);
  convert(struct_v.obj_update_flag, ros_v.obj_update_flag, type);
  convert(struct_v.obj_motion_pattern, ros_v.obj_motion_pattern, type);
  convert(struct_v.obstacle_prob, ros_v.obstacle_prob, type);
}

template <typename T2>
void convert(iflyauto::RadarPerceptionObjectsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  convert(struct_v.sensor_type, ros_v.sensor_type, type);
  convert(struct_v.object_list_size, ros_v.object_list_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.object_list_size >= 0 && struct_v.object_list_size <= RADAR_PERCEPTION_OBJ_MAX_NUM) {
      ros_v.object_list.resize(struct_v.object_list_size);
    } else {
      std::cout << "convert/radar_perception_objects_c.h:" << __LINE__ 
                << " [convert][TO_ROS] object_list_size=" << struct_v.object_list_size 
                << " not in range RADAR_PERCEPTION_OBJ_MAX_NUM=" << RADAR_PERCEPTION_OBJ_MAX_NUM 
                << std::endl;
      ros_v.object_list_size = RADAR_PERCEPTION_OBJ_MAX_NUM;
      ros_v.object_list.resize(RADAR_PERCEPTION_OBJ_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.object_list.size(); i0++) {
      convert(struct_v.object_list[i0], ros_v.object_list[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.object_list_size > RADAR_PERCEPTION_OBJ_MAX_NUM || ros_v.object_list_size < 0 || ros_v.object_list.size() > RADAR_PERCEPTION_OBJ_MAX_NUM) {
      std::cout << "convert/radar_perception_objects_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] object_list_size=" << ros_v.object_list_size 
                << " ros_v.object_list.size()=" << ros_v.object_list.size()
                << " not in range RADAR_PERCEPTION_OBJ_MAX_NUM=" << RADAR_PERCEPTION_OBJ_MAX_NUM 
                << std::endl;
    }
    if (ros_v.object_list.size() > RADAR_PERCEPTION_OBJ_MAX_NUM) {
      for (size_t i0 = 0; i0 < RADAR_PERCEPTION_OBJ_MAX_NUM; i0++) {
        convert(struct_v.object_list[i0], ros_v.object_list[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.object_list.size(); i0++) {
        convert(struct_v.object_list[i0], ros_v.object_list[i0], type);
      }
    }
  }
  //
  convert(struct_v.crc, ros_v.crc, type);
}

