#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FmInfo.h"
#include "struct_msgs_v2_10/FmInfo.h"
#include "struct_msgs/FmInfoArray.h"
#include "struct_msgs_v2_10/FmInfoArray.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FmInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.alarmId, ros_v.alarmId, type);
	convert(old_ros_v.alarmObj, ros_v.alarmObj, type);
	convert(old_ros_v.alarmCount, ros_v.alarmCount, type);
	convert(old_ros_v.clss, ros_v.clss, type);
	convert(old_ros_v.level, ros_v.level, type);
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.time, ros_v.time, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.desc[i], ros_v.desc[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FmInfoArray &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.alarm_counts, ros_v.alarm_counts, type);
	ros_v.fmInfos.resize(old_ros_v.fmInfos.size());
	for (int i = 0; i < ros_v.fmInfos.size(); i++) {
	    convert(old_ros_v.fmInfos[i], ros_v.fmInfos[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_alarm_info_fm_a_service_converter, "/iflytek/alarm_info/fm_a_service", FmInfo);
REG_CONVERT_SINGLE(_iflytek_alarm_info_fm_b_service_converter, "/iflytek/alarm_info/fm_b_service", FmInfo);
