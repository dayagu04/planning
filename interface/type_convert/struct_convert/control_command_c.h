#pragma once

#include "base_convert.h"
#include "c/control_command_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ControlTrajectory &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.control_result_points_size, ros_v.control_result_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.control_result_points_size >= 0 && struct_v.control_result_points_size <= CONTROL_RESULT_POINTS_MAX_NUM) {
      ros_v.control_result_points.resize(struct_v.control_result_points_size);
    } else {
      std::cout << "convert/control_command_c.h:" << __LINE__ 
                << " [convert][TO_ROS] control_result_points_size=" << struct_v.control_result_points_size 
                << " not in range CONTROL_RESULT_POINTS_MAX_NUM=" << CONTROL_RESULT_POINTS_MAX_NUM 
                << std::endl;
      ros_v.control_result_points_size = CONTROL_RESULT_POINTS_MAX_NUM;
      ros_v.control_result_points.resize(CONTROL_RESULT_POINTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.control_result_points.size(); i0++) {
      convert(struct_v.control_result_points[i0], ros_v.control_result_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.control_result_points_size > CONTROL_RESULT_POINTS_MAX_NUM || ros_v.control_result_points_size < 0 || ros_v.control_result_points.size() > CONTROL_RESULT_POINTS_MAX_NUM) {
      std::cout << "convert/control_command_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] control_result_points_size=" << ros_v.control_result_points_size 
                << " ros_v.control_result_points.size()=" << ros_v.control_result_points.size()
                << " not in range CONTROL_RESULT_POINTS_MAX_NUM=" << CONTROL_RESULT_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.control_result_points.size() > CONTROL_RESULT_POINTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CONTROL_RESULT_POINTS_MAX_NUM; i0++) {
        convert(struct_v.control_result_points[i0], ros_v.control_result_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.control_result_points.size(); i0++) {
        convert(struct_v.control_result_points[i0], ros_v.control_result_points[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::LatControlMode &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.lat_control_mode, ros_v.lat_control_mode, type);
}

template <typename T2>
void convert(iflyauto::LonControlMode &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.lon_control_mode, ros_v.lon_control_mode, type);
}

template <typename T2>
void convert(iflyauto::ControlStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.control_status_type, ros_v.control_status_type, type);
}

template <typename T2>
void convert(iflyauto::ControlOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.lat_control_mode, ros_v.lat_control_mode, type);
  convert(struct_v.steering, ros_v.steering, type);
  convert(struct_v.steering_rate, ros_v.steering_rate, type);
  convert(struct_v.steering_torque, ros_v.steering_torque, type);
  convert(struct_v.lon_control_mode, ros_v.lon_control_mode, type);
  convert(struct_v.throttle, ros_v.throttle, type);
  convert(struct_v.brake, ros_v.brake, type);
  convert(struct_v.speed, ros_v.speed, type);
  convert(struct_v.acceleration, ros_v.acceleration, type);
  convert(struct_v.axle_torque, ros_v.axle_torque, type);
  convert(struct_v.stop_distance, ros_v.stop_distance, type);
  convert(struct_v.gear_command_value, ros_v.gear_command_value, type);
  convert(struct_v.turn_signal_cmd, ros_v.turn_signal_cmd, type);
  convert(struct_v.light_signal_cmd, ros_v.light_signal_cmd, type);
  convert(struct_v.horn_signal_cmd, ros_v.horn_signal_cmd, type);
  convert(struct_v.rear_view_mirror_signal_cmd, ros_v.rear_view_mirror_signal_cmd, type);
  convert(struct_v.control_trajectory, ros_v.control_trajectory, type);
  convert(struct_v.control_status, ros_v.control_status, type);
  convert(struct_v.standstill_request, ros_v.standstill_request, type);
  convert(struct_v.driver_off_request, ros_v.driver_off_request, type);
  convert(struct_v.axle_torque_request, ros_v.axle_torque_request, type);
  convert(struct_v.acceleration_request, ros_v.acceleration_request, type);
  convert(struct_v.acc_cmd, ros_v.acc_cmd, type);
  convert(struct_v.epb_cmd, ros_v.epb_cmd, type);
  convert(struct_v.steering_torque_max, ros_v.steering_torque_max, type);
  convert(struct_v.steering_torque_min, ros_v.steering_torque_min, type);
  convert(struct_v.amap_request_flag, ros_v.amap_request_flag, type);
  convert(struct_v.amap_trq_limit_max, ros_v.amap_trq_limit_max, type);
  convert(struct_v.meb_request_status, ros_v.meb_request_status, type);
  convert(struct_v.meb_request_value, ros_v.meb_request_value, type);
}

