#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CalibInfo.h"
#include "struct_msgs_v2_10/CalibInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CalibInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.calib_type, ros_v.calib_type, type);
	convert(old_ros_v.calib_status, ros_v.calib_status, type);
	convert(old_ros_v.action, ros_v.action, type);
	convert(old_ros_v.time, ros_v.time, type);
	for (int i = 0; i < 128; i++) {
	    convert(old_ros_v.calib_sensors[i], ros_v.calib_sensors[i], type);
	}
	for (int i = 0; i < 128; i++) {
	    convert(old_ros_v.desc[i], ros_v.desc[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_calibration_calib_info_converter, "/iflytek/calibration/calib_info", CalibInfo);
