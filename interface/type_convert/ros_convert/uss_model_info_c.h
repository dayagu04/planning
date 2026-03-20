#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/UssModelCoordinateDataType.h"
#include "struct_msgs_v2_10/UssModelCoordinateDataType.h"
#include "struct_msgs/UssModelInfo.h"
#include "struct_msgs_v2_10/UssModelInfo.h"
#include "struct_msgs/UssModelPosition.h"
#include "struct_msgs_v2_10/UssModelPosition.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssModelCoordinateDataType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 400; i++) {
	    convert(old_ros_v.obj_pt_global[i], ros_v.obj_pt_global[i], type);
	}
	for (int i = 0; i < 400; i++) {
	    convert(old_ros_v.obj_pt_local[i], ros_v.obj_pt_local[i], type);
	}
	for (int i = 0; i < 400; i++) {
	    convert(old_ros_v.obj_type0[i], ros_v.obj_type0[i], type);
	}
	convert(old_ros_v.wr_index, ros_v.wr_index, type);
	convert(old_ros_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssModelInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.uss_wave_distance[i], ros_v.uss_wave_distance[i], type);
	}
	convert(old_ros_v.uss_wave_position, ros_v.uss_wave_position, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.dis_from_car_to_obj[i], ros_v.dis_from_car_to_obj[i], type);
	}
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.out_line_dataori[i], ros_v.out_line_dataori[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssModelPosition &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.position, ros_v.position, type);
	convert(old_ros_v.yaw, ros_v.yaw, type);
}

