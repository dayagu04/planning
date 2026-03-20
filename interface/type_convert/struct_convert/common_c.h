#pragma once

#include "base_convert.h"
#include "c/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::Point2si &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
}

template <typename T2>
void convert(iflyauto::Point2f &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
}

template <typename T2>
void convert(iflyauto::Point2d &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
}

template <typename T2>
void convert(iflyauto::Point3si &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::Point3f &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::Point3d &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::Shape3f &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.width, ros_v.width, type);
  convert(struct_v.height, ros_v.height, type);
}

template <typename T2>
void convert(iflyauto::Shape3d &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.width, ros_v.width, type);
  convert(struct_v.height, ros_v.height, type);
}

template <typename T2>
void convert(iflyauto::PointENU &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::TileId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
}

template <typename T2>
void convert(iflyauto::UrId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
}

template <typename T2>
void convert(iflyauto::FloorId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
}

template <typename T2>
void convert(iflyauto::PointLLH &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lon, ros_v.lon, type);
  convert(struct_v.lat, ros_v.lat, type);
  convert(struct_v.height, ros_v.height, type);
}

template <typename T2>
void convert(iflyauto::Quaternion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.w, ros_v.w, type);
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::InputHistoryInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.input_type, ros_v.input_type, type);
  convert(struct_v.seq, ros_v.seq, type);
  convert(struct_v.stamp, ros_v.stamp, type);
}

template <typename T2>
void convert(iflyauto::MsgMeta &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.start_time, ros_v.start_time, type);
  for (size_t i0 = 0; i0 < ros_v.version.size(); i0++) {
	  convert(struct_v.version[i0], ros_v.version[i0], type);
  }
  convert(struct_v.input_list_size, ros_v.input_list_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.input_list_size >= 0 && struct_v.input_list_size <= INPUT_HISTORY_TIMESTAMP_MAX_NUM) {
      ros_v.input_list.resize(struct_v.input_list_size);
    } else {
      std::cout << "convert/common_c.h:" << __LINE__ 
                << " [convert][TO_ROS] input_list_size=" << struct_v.input_list_size 
                << " not in range INPUT_HISTORY_TIMESTAMP_MAX_NUM=" << INPUT_HISTORY_TIMESTAMP_MAX_NUM 
                << std::endl;
      ros_v.input_list_size = INPUT_HISTORY_TIMESTAMP_MAX_NUM;
      ros_v.input_list.resize(INPUT_HISTORY_TIMESTAMP_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.input_list.size(); i1++) {
      convert(struct_v.input_list[i1], ros_v.input_list[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.input_list_size > INPUT_HISTORY_TIMESTAMP_MAX_NUM || ros_v.input_list_size < 0 || ros_v.input_list.size() > INPUT_HISTORY_TIMESTAMP_MAX_NUM) {
      std::cout << "convert/common_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] input_list_size=" << ros_v.input_list_size 
                << " ros_v.input_list.size()=" << ros_v.input_list.size()
                << " not in range INPUT_HISTORY_TIMESTAMP_MAX_NUM=" << INPUT_HISTORY_TIMESTAMP_MAX_NUM 
                << std::endl;
    }
    if (ros_v.input_list.size() > INPUT_HISTORY_TIMESTAMP_MAX_NUM) {
      for (size_t i1 = 0; i1 < INPUT_HISTORY_TIMESTAMP_MAX_NUM; i1++) {
        convert(struct_v.input_list[i1], ros_v.input_list[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.input_list.size(); i1++) {
        convert(struct_v.input_list[i1], ros_v.input_list[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::SensorMeta &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.stamp, ros_v.stamp, type);
}

template <typename T2>
void convert(iflyauto::CommonCameraPerceptionInputTimestamp &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.front_isp_timestamp, ros_v.front_isp_timestamp, type);
  convert(struct_v.rear_isp_timestamp, ros_v.rear_isp_timestamp, type);
  convert(struct_v.fl_isp_timestamp, ros_v.fl_isp_timestamp, type);
  convert(struct_v.rl_isp_timestamp, ros_v.rl_isp_timestamp, type);
  convert(struct_v.fr_isp_timestamp, ros_v.fr_isp_timestamp, type);
  convert(struct_v.rr_isp_timestamp, ros_v.rr_isp_timestamp, type);
  convert(struct_v.vehicle_service_timestamp, ros_v.vehicle_service_timestamp, type);
  convert(struct_v.pbox_imu_timestamp, ros_v.pbox_imu_timestamp, type);
}

template <typename T2>
void convert(iflyauto::MsgHeader &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.seq, ros_v.seq, type);
  convert(struct_v.stamp, ros_v.stamp, type);
}

template <typename T2>
void convert(iflyauto::Obstacle &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.light_type, ros_v.light_type, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.relative_position, ros_v.relative_position, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_acceleration, ros_v.relative_acceleration, type);
  convert(struct_v.relative_center_position, ros_v.relative_center_position, type);
  convert(struct_v.relative_heading_angle, ros_v.relative_heading_angle, type);
  convert(struct_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
  convert(struct_v.position, ros_v.position, type);
  convert(struct_v.velocity, ros_v.velocity, type);
  convert(struct_v.acceleration, ros_v.acceleration, type);
  convert(struct_v.center_position, ros_v.center_position, type);
  convert(struct_v.heading_angle, ros_v.heading_angle, type);
  convert(struct_v.heading_angle_rate, ros_v.heading_angle_rate, type);
  convert(struct_v.is_dangerous, ros_v.is_dangerous, type);
  convert(struct_v.dangerous_conf, ros_v.dangerous_conf, type);
}

template <typename T2>
void convert(iflyauto::LaneMergeSplitPointData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.distance, ros_v.distance, type);
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.is_split, ros_v.is_split, type);
  convert(struct_v.is_continue, ros_v.is_continue, type);
  convert(struct_v.orientation, ros_v.orientation, type);
  convert(struct_v.point, ros_v.point, type);
}

template <typename T2>
void convert(iflyauto::UtcTime &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.year, ros_v.year, type);
  convert(struct_v.month, ros_v.month, type);
  convert(struct_v.day, ros_v.day, type);
  convert(struct_v.hour, ros_v.hour, type);
  convert(struct_v.minute, ros_v.minute, type);
  convert(struct_v.second, ros_v.second, type);
  convert(struct_v.milli_second, ros_v.milli_second, type);
  convert(struct_v.time_accuracy, ros_v.time_accuracy, type);
}

template <typename T2>
void convert(iflyauto::LaneMarkMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.lane_mark, ros_v.lane_mark, type);
}

template <typename T2>
void convert(iflyauto::LaneTypeMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.type, ros_v.type, type);
}

