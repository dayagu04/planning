#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/PostSaleCalibActiveInfo.h"
#include "struct_msgs_v2_10/PostSaleCalibActiveInfo.h"
#include "struct_msgs/PostSaleCalibActiveInfoResult.h"
#include "struct_msgs_v2_10/PostSaleCalibActiveInfoResult.h"
#include "struct_msgs/PostSaleCalibGetCalibResReqInfo.h"
#include "struct_msgs_v2_10/PostSaleCalibGetCalibResReqInfo.h"
#include "struct_msgs/PostSaleCalibResults.h"
#include "struct_msgs_v2_10/PostSaleCalibResults.h"
#include "struct_msgs/PostSaleCalibStopReqInfo.h"
#include "struct_msgs_v2_10/PostSaleCalibStopReqInfo.h"
#include "struct_msgs/PostSaleCalibStopResult.h"
#include "struct_msgs_v2_10/PostSaleCalibStopResult.h"
#include "struct_msgs/PostSaleSensorCalibResult.h"
#include "struct_msgs_v2_10/PostSaleSensorCalibResult.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibActiveInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.calib_active, ros_v.calib_active, type);
	for (int i = 0; i < 100; i++) {
	    convert(old_ros_v.calib_sensors[i], ros_v.calib_sensors[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibActiveInfoResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.result, ros_v.result, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibGetCalibResReqInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.req_calib_res, ros_v.req_calib_res, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibResults &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.result, ros_v.result, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.calib_results[i], ros_v.calib_results[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibStopReqInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.stop_calib, ros_v.stop_calib, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleCalibStopResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.result, ros_v.result, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostSaleSensorCalibResult &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.result, ros_v.result, type);
	convert(old_ros_v.delta_x, ros_v.delta_x, type);
	convert(old_ros_v.delta_y, ros_v.delta_y, type);
	convert(old_ros_v.delta_z, ros_v.delta_z, type);
	convert(old_ros_v.delta_roll, ros_v.delta_roll, type);
	convert(old_ros_v.delta_yaw, ros_v.delta_yaw, type);
	convert(old_ros_v.delta_pitch, ros_v.delta_pitch, type);
}

