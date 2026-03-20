#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/IFLYAllDtcState.h"
#include "struct_msgs_v2_10/IFLYAllDtcState.h"
#include "struct_msgs/IFLYDtcCode.h"
#include "struct_msgs_v2_10/IFLYDtcCode.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAllDtcState &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.dtc_code_size, ros_v.dtc_code_size, type);
	ros_v.dtc_code.resize(old_ros_v.dtc_code.size());
	for (int i = 0; i < ros_v.dtc_code.size(); i++) {
	    convert(old_ros_v.dtc_code[i], ros_v.dtc_code[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYDtcCode &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.code[i], ros_v.code[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_dtc_info_fm_a_service_converter, "/iflytek/dtc_info/fm_a_service", IFLYAllDtcState);
