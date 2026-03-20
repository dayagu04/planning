#pragma once

#include "base_convert.h"
#include "c/sensor_imu_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::IFLYIMU &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.imu_name.size(); i0++) {
	  convert(struct_v.imu_name[i0], ros_v.imu_name[i0], type);
  }
  convert(struct_v.temperature_val, ros_v.temperature_val, type);
  convert(struct_v.gyro_selftest_result, ros_v.gyro_selftest_result, type);
  convert(struct_v.acc_selftest_result, ros_v.acc_selftest_result, type);
  convert(struct_v.acc_val, ros_v.acc_val, type);
  convert(struct_v.angular_rate_val, ros_v.angular_rate_val, type);
}

