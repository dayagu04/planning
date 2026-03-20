#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/HmiMcuOut.h"
#include "struct_msgs_v2_10/HmiMcuOut.h"
#include "struct_msgs/HmiPdcInfo.h"
#include "struct_msgs_v2_10/HmiPdcInfo.h"
#include "struct_msgs/PdcRadarInfo.h"
#include "struct_msgs_v2_10/PdcRadarInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiMcuOut &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.adas_out_info, ros_v.adas_out_info, type);
	convert(old_ros_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiPdcInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.pdc_sys_sts, ros_v.pdc_sys_sts, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.pdc_info[i], ros_v.pdc_info[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PdcRadarInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.obj_dis, ros_v.obj_dis, type);
	convert(old_ros_v.radar_pos, ros_v.radar_pos, type);
	convert(old_ros_v.warning_level, ros_v.warning_level, type);
}

