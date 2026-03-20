#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.6/localization_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_6::EulerAngle &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.yaw, ros_v.yaw, type);
  convert(struct_v.pitch, ros_v.pitch, type);
  convert(struct_v.roll, ros_v.roll, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_6::Pose &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.enu_position, ros_v.enu_position, type);
  convert(struct_v.llh_position, ros_v.llh_position, type);
  convert(struct_v.local_position, ros_v.local_position, type);
  convert(struct_v.orientation, ros_v.orientation, type);
  convert(struct_v.linear_acceleration, ros_v.linear_acceleration, type);
  convert(struct_v.angular_velocity, ros_v.angular_velocity, type);
  convert(struct_v.heading, ros_v.heading, type);
  convert(struct_v.euler_angles, ros_v.euler_angles, type);
  convert(struct_v.linear_velocity_from_wheel, ros_v.linear_velocity_from_wheel, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_6::MsfStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.msf_status, ros_v.msf_status, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_6::OddStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.odd_status, ros_v.odd_status, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_6::Uncertainty &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.var_x, ros_v.var_x, type);
  convert(struct_v.var_y, ros_v.var_y, type);
  convert(struct_v.var_theta, ros_v.var_theta, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_6::LocalizationEstimate &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.pose, ros_v.pose, type);
  convert(struct_v.msf_status, ros_v.msf_status, type);
  convert(struct_v.uncertainty, ros_v.uncertainty, type);
}

