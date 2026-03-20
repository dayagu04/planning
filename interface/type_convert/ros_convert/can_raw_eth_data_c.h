#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CanAscData.h"
#include "struct_msgs_v2_10/CanAscData.h"
#include "struct_msgs/CanRawEthData.h"
#include "struct_msgs_v2_10/CanRawEthData.h"
#include "struct_msgs/CanRawMsg.h"
#include "struct_msgs_v2_10/CanRawMsg.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CanAscData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	for (int i = 0; i < 512; i++) {
	    convert(old_ros_v.asc_header[i], ros_v.asc_header[i], type);
	}
	convert(old_ros_v.asc_data_size, ros_v.asc_data_size, type);
	ros_v.asc_data.resize(old_ros_v.asc_data.size());
	for (int i = 0; i < ros_v.asc_data.size(); i++) {
	    convert(old_ros_v.asc_data[i], ros_v.asc_data[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CanRawEthData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.canRawMsg_size, ros_v.canRawMsg_size, type);
	ros_v.canRawMsg.resize(old_ros_v.canRawMsg.size());
	for (int i = 0; i < ros_v.canRawMsg.size(); i++) {
	    convert(old_ros_v.canRawMsg[i], ros_v.canRawMsg[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CanRawMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.canId, ros_v.canId, type);
	convert(old_ros_v.can_data_size, ros_v.can_data_size, type);
	ros_v.can_data.resize(old_ros_v.can_data.size());
	for (int i = 0; i < ros_v.can_data.size(); i++) {
	    convert(old_ros_v.can_data[i], ros_v.can_data[i], type);
	}
	for (int i = 0; i < 3; i++) {
	    convert(old_ros_v.resv[i], ros_v.resv[i], type);
	}
	convert(old_ros_v.timestamp, ros_v.timestamp, type);
}

REG_CONVERT_SINGLE(_iflytek_dc_can_raw_converter, "/iflytek/dc/can/raw", CanRawEthData);
