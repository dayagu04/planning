#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ApaSlotOutlineCoordinateDataType.h"
#include "struct_msgs_v2_10/ApaSlotOutlineCoordinateDataType.h"
#include "struct_msgs/UssObjPointInfo.h"
#include "struct_msgs_v2_10/UssObjPointInfo.h"
#include "struct_msgs/UssPerceptInfo.h"
#include "struct_msgs_v2_10/UssPerceptInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ApaSlotOutlineCoordinateDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.wr_index, ros_v.wr_index, type);
	convert(old_ros_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
	for (int i = 0; i < 200; i++) {
	    convert(old_ros_v.obj_pt_global[i], ros_v.obj_pt_global[i], type);
	}
	for (int i = 0; i < 200; i++) {
	    convert(old_ros_v.obj_pt_local[i], ros_v.obj_pt_local[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssObjPointInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.obj_point_type, ros_v.obj_point_type, type);
	convert(old_ros_v.obj_point_xpos, ros_v.obj_point_xpos, type);
	convert(old_ros_v.obj_point_ypos, ros_v.obj_point_ypos, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPerceptInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.dis_from_car_to_obj[i], ros_v.dis_from_car_to_obj[i], type);
	}
	for (int i = 0; i < 1; i++) {
	    convert(old_ros_v.out_line_dataori[i], ros_v.out_line_dataori[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_fusion_uss_perception_info_converter, "/iflytek/fusion/uss_perception_info", UssPerceptInfo);
