#pragma once

#include "base_convert.h"
#include "c/post_sale_calibration_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::PostSaleCalibActiveInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.calib_active, ros_v.calib_active, type);
  for (size_t i0 = 0; i0 < ros_v.calib_sensors.size(); i0++) {
	  convert(struct_v.calib_sensors[i0], ros_v.calib_sensors[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::PostSaleCalibActiveInfoResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.result, ros_v.result, type);
}

template <typename T2>
void convert(iflyauto::PostSaleCalibStopReqInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.stop_calib, ros_v.stop_calib, type);
}

template <typename T2>
void convert(iflyauto::PostSaleCalibStopResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.result, ros_v.result, type);
}

template <typename T2>
void convert(iflyauto::PostSaleCalibGetCalibResReqInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.req_calib_res, ros_v.req_calib_res, type);
}

template <typename T2>
void convert(iflyauto::PostSaleSensorCalibResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.result, ros_v.result, type);
  for (size_t i0 = 0; i0 < ros_v.sensor_name.size(); i0++) {
	  convert(struct_v.sensor_name[i0], ros_v.sensor_name[i0], type);
  }
  convert(struct_v.delta_x, ros_v.delta_x, type);
  convert(struct_v.delta_y, ros_v.delta_y, type);
  convert(struct_v.delta_z, ros_v.delta_z, type);
  convert(struct_v.delta_roll, ros_v.delta_roll, type);
  convert(struct_v.delta_yaw, ros_v.delta_yaw, type);
  convert(struct_v.delta_pitch, ros_v.delta_pitch, type);
}

template <typename T2>
void convert(iflyauto::PostSaleCalibResults &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.result, ros_v.result, type);
  for (size_t i0 = 0; i0 < ros_v.calib_results.size(); i0++) {
	  convert(struct_v.calib_results[i0], ros_v.calib_results[i0], type);
  }
}

