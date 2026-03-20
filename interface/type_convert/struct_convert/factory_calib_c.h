#pragma once

#include "base_convert.h"
#include "c/factory_calib_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FactoryCalibCarModeInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.car_mode, ros_v.car_mode, type);
}

template <typename T2>
void convert(iflyauto::FactoryCalibActiveInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.calib_active, ros_v.calib_active, type);
  for (size_t i0 = 0; i0 < ros_v.calib_sensors.size(); i0++) {
	  convert(struct_v.calib_sensors[i0], ros_v.calib_sensors[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::FactoryCalibActiveInfoResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.result, ros_v.result, type);
}

template <typename T2>
void convert(iflyauto::FactoryCalibGetCalibResReqInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.req_calib_res, ros_v.req_calib_res, type);
}

template <typename T2>
void convert(iflyauto::FactorySensorCalibResult &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.result, ros_v.result, type);
  for (size_t i0 = 0; i0 < ros_v.sensor_name.size(); i0++) {
	  convert(struct_v.sensor_name[i0], ros_v.sensor_name[i0], type);
  }
  convert(struct_v.install_error_rx, ros_v.install_error_rx, type);
  convert(struct_v.install_error_ry, ros_v.install_error_ry, type);
  convert(struct_v.install_error_rz, ros_v.install_error_rz, type);
  convert(struct_v.install_error_tx, ros_v.install_error_tx, type);
  convert(struct_v.install_error_ty, ros_v.install_error_ty, type);
  convert(struct_v.install_error_tz, ros_v.install_error_tz, type);
}

template <typename T2>
void convert(iflyauto::FactoryCalibResults &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.result, ros_v.result, type);
  for (size_t i0 = 0; i0 < ros_v.calib_results.size(); i0++) {
	  convert(struct_v.calib_results[i0], ros_v.calib_results[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::FactoryCalibIntriParam &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.result, ros_v.result, type);
  convert(struct_v.name, ros_v.name, type);
  convert(struct_v.model, ros_v.model, type);
  for (size_t i0 = 0; i0 < ros_v.hardware_info.size(); i0++) {
	  convert(struct_v.hardware_info[i0], ros_v.hardware_info[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.sn.size(); i1++) {
	  convert(struct_v.sn[i1], ros_v.sn[i1], type);
  }
  convert(struct_v.fx, ros_v.fx, type);
  convert(struct_v.fy, ros_v.fy, type);
  convert(struct_v.cx, ros_v.cx, type);
  convert(struct_v.cy, ros_v.cy, type);
  convert(struct_v.width, ros_v.width, type);
  convert(struct_v.height, ros_v.height, type);
  convert(struct_v.distortion_param_size, ros_v.distortion_param_size, type);
  for (size_t i2 = 0; i2 < ros_v.distortion_param_array.size(); i2++) {
	  convert(struct_v.distortion_param_array[i2], ros_v.distortion_param_array[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::FactoryCalibSvcIntriParamReqInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.svc_intri_param_req, ros_v.svc_intri_param_req, type);
}

template <typename T2>
void convert(iflyauto::FactoryCalibSvcIntriParamArray &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.size, ros_v.size, type);
  for (size_t i0 = 0; i0 < ros_v.parameter.size(); i0++) {
	  convert(struct_v.parameter[i0], ros_v.parameter[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::FactoryCalibPanoramaIntriParamReqInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.panorama_intri_param_req, ros_v.panorama_intri_param_req, type);
}

template <typename T2>
void convert(iflyauto::FactoryCalibPanoramaIntriParamArray &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.size, ros_v.size, type);
  for (size_t i0 = 0; i0 < ros_v.parameter.size(); i0++) {
	  convert(struct_v.parameter[i0], ros_v.parameter[i0], type);
  }
}

