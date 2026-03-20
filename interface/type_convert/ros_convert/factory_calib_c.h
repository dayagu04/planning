#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FactoryCalibActiveInfo.h"
#include "struct_msgs_v2_10/FactoryCalibActiveInfo.h"
#include "struct_msgs/FactoryCalibActiveInfoResult.h"
#include "struct_msgs_v2_10/FactoryCalibActiveInfoResult.h"
#include "struct_msgs/FactoryCalibCarModeInfo.h"
#include "struct_msgs_v2_10/FactoryCalibCarModeInfo.h"
#include "struct_msgs/FactoryCalibGetCalibResReqInfo.h"
#include "struct_msgs_v2_10/FactoryCalibGetCalibResReqInfo.h"
#include "struct_msgs/FactoryCalibIntriParam.h"
#include "struct_msgs_v2_10/FactoryCalibIntriParam.h"
#include "struct_msgs/FactoryCalibPanoramaIntriParamArray.h"
#include "struct_msgs_v2_10/FactoryCalibPanoramaIntriParamArray.h"
#include "struct_msgs/FactoryCalibPanoramaIntriParamReqInfo.h"
#include "struct_msgs_v2_10/FactoryCalibPanoramaIntriParamReqInfo.h"
#include "struct_msgs/FactoryCalibResults.h"
#include "struct_msgs_v2_10/FactoryCalibResults.h"
#include "struct_msgs/FactoryCalibSvcIntriParamArray.h"
#include "struct_msgs_v2_10/FactoryCalibSvcIntriParamArray.h"
#include "struct_msgs/FactoryCalibSvcIntriParamReqInfo.h"
#include "struct_msgs_v2_10/FactoryCalibSvcIntriParamReqInfo.h"
#include "struct_msgs/FactorySensorCalibResult.h"
#include "struct_msgs_v2_10/FactorySensorCalibResult.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibActiveInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.calib_active, ros_v.calib_active, type);
	for (int i = 0; i < 100; i++) {
	    convert(old_ros_v.calib_sensors[i], ros_v.calib_sensors[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibActiveInfoResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.result, ros_v.result, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibCarModeInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.car_mode, ros_v.car_mode, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibGetCalibResReqInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.req_calib_res, ros_v.req_calib_res, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibIntriParam &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.result, ros_v.result, type);
	convert(old_ros_v.name, ros_v.name, type);
	convert(old_ros_v.model, ros_v.model, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.hardware_info[i], ros_v.hardware_info[i], type);
	}
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.sn[i], ros_v.sn[i], type);
	}
	convert(old_ros_v.fx, ros_v.fx, type);
	convert(old_ros_v.fy, ros_v.fy, type);
	convert(old_ros_v.cx, ros_v.cx, type);
	convert(old_ros_v.cy, ros_v.cy, type);
	convert(old_ros_v.width, ros_v.width, type);
	convert(old_ros_v.height, ros_v.height, type);
	convert(old_ros_v.distortion_param_size, ros_v.distortion_param_size, type);
	for (int i = 0; i < 20; i++) {
	    convert(old_ros_v.distortion_param_array[i], ros_v.distortion_param_array[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibPanoramaIntriParamArray &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.size, ros_v.size, type);
	for (int i = 0; i < 7; i++) {
	    convert(old_ros_v.parameter[i], ros_v.parameter[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibPanoramaIntriParamReqInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.panorama_intri_param_req, ros_v.panorama_intri_param_req, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibResults &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.result, ros_v.result, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.calib_results[i], ros_v.calib_results[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibSvcIntriParamArray &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.size, ros_v.size, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.parameter[i], ros_v.parameter[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactoryCalibSvcIntriParamReqInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.svc_intri_param_req, ros_v.svc_intri_param_req, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FactorySensorCalibResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.result, ros_v.result, type);
	convert(old_ros_v.install_error_rx, ros_v.install_error_rx, type);
	convert(old_ros_v.install_error_ry, ros_v.install_error_ry, type);
	convert(old_ros_v.install_error_rz, ros_v.install_error_rz, type);
	convert(old_ros_v.install_error_tx, ros_v.install_error_tx, type);
	convert(old_ros_v.install_error_ty, ros_v.install_error_ty, type);
	convert(old_ros_v.install_error_tz, ros_v.install_error_tz, type);
}

REG_CONVERT_SINGLE(_iflytek_hmi_factory_mode_converter, "/iflytek/hmi/factory_mode", FactoryCalibCarModeInfo);
