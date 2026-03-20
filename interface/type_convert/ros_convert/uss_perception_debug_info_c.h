#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AusspsListUss.h"
#include "struct_msgs_v2_10/AusspsListUss.h"
#include "struct_msgs/AusspsUss.h"
#include "struct_msgs_v2_10/AusspsUss.h"
#include "struct_msgs/CornerPoints.h"
#include "struct_msgs_v2_10/CornerPoints.h"
#include "struct_msgs/SlotInfoDataType.h"
#include "struct_msgs_v2_10/SlotInfoDataType.h"
#include "struct_msgs/SlotParameterType.h"
#include "struct_msgs_v2_10/SlotParameterType.h"
#include "struct_msgs/UssPerceptDebugInfo.h"
#include "struct_msgs_v2_10/UssPerceptDebugInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AusspsListUss &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.timestamp, ros_v.timestamp, type);
	convert(old_ros_v.parking_slot_size, ros_v.parking_slot_size, type);
	for (int i = 0; i < 6; i++) {
	    convert(old_ros_v.parking_slots[i], ros_v.parking_slots[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AusspsUss &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.sita, ros_v.sita, type);
	convert(old_ros_v.slot_side, ros_v.slot_side, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.border_type[i], ros_v.border_type[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.corner_point[i], ros_v.corner_point[i], type);
	}
	convert(old_ros_v.slot_type, ros_v.slot_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CornerPoints &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.wr_index, ros_v.wr_index, type);
	convert(old_ros_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
	for (int i = 0; i < 200; i++) {
	    convert(old_ros_v.obj_pt[i], ros_v.obj_pt[i], type);
	}
	for (int i = 0; i < 200; i++) {
	    convert(old_ros_v.dis_from_car_to_obj[i], ros_v.dis_from_car_to_obj[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SlotInfoDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.slot_num, ros_v.slot_num, type);
	convert(old_ros_v.slot_confirm_seq, ros_v.slot_confirm_seq, type);
	convert(old_ros_v.slot_prev_id, ros_v.slot_prev_id, type);
	for (int i = 0; i < 6; i++) {
	    convert(old_ros_v.slot_par[i], ros_v.slot_par[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SlotParameterType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.slot_type, ros_v.slot_type, type);
	convert(old_ros_v.slot_type_confirm, ros_v.slot_type_confirm, type);
	convert(old_ros_v.slot_start_index, ros_v.slot_start_index, type);
	convert(old_ros_v.slot_end_index, ros_v.slot_end_index, type);
	convert(old_ros_v.slot_cal_dis_car_to_obj1, ros_v.slot_cal_dis_car_to_obj1, type);
	convert(old_ros_v.slot_cal_dis_car_to_obj2, ros_v.slot_cal_dis_car_to_obj2, type);
	convert(old_ros_v.slot_length_detected1, ros_v.slot_length_detected1, type);
	convert(old_ros_v.slot_length_detected2, ros_v.slot_length_detected2, type);
	convert(old_ros_v.slot_length_detected3, ros_v.slot_length_detected3, type);
	convert(old_ros_v.slot_length_total, ros_v.slot_length_total, type);
	convert(old_ros_v.slot_depth_detected1, ros_v.slot_depth_detected1, type);
	convert(old_ros_v.slot_depth_detected2, ros_v.slot_depth_detected2, type);
	convert(old_ros_v.slot_depth_detected3, ros_v.slot_depth_detected3, type);
	convert(old_ros_v.slot_length, ros_v.slot_length, type);
	convert(old_ros_v.slot_depth_parallel, ros_v.slot_depth_parallel, type);
	convert(old_ros_v.slot_depth, ros_v.slot_depth, type);
	convert(old_ros_v.sub_slot_depth, ros_v.sub_slot_depth, type);
	convert(old_ros_v.dis_car_to_obj1_by_passing_slot, ros_v.dis_car_to_obj1_by_passing_slot, type);
	convert(old_ros_v.dis_car_to_obj2_by_passing_slot, ros_v.dis_car_to_obj2_by_passing_slot, type);
	convert(old_ros_v.car_pass_the_slot_end_pt_distance, ros_v.car_pass_the_slot_end_pt_distance, type);
	convert(old_ros_v.car_pass_the_slot_start_pt_distance, ros_v.car_pass_the_slot_start_pt_distance, type);
	convert(old_ros_v.apa_slot_detection_compensate_length_head, ros_v.apa_slot_detection_compensate_length_head, type);
	convert(old_ros_v.apa_slot_detection_compensate_length_tail, ros_v.apa_slot_detection_compensate_length_tail, type);
	convert(old_ros_v.obj1_width, ros_v.obj1_width, type);
	convert(old_ros_v.obj2_width, ros_v.obj2_width, type);
	convert(old_ros_v.obj1_width_confirm, ros_v.obj1_width_confirm, type);
	convert(old_ros_v.obj2_width_confirm, ros_v.obj2_width_confirm, type);
	convert(old_ros_v.obj1_type, ros_v.obj1_type, type);
	convert(old_ros_v.obj2_type, ros_v.obj2_type, type);
	convert(old_ros_v.obj1_start_pt_index, ros_v.obj1_start_pt_index, type);
	convert(old_ros_v.obj1_end_pt_index, ros_v.obj1_end_pt_index, type);
	convert(old_ros_v.obj2_start_pt_index, ros_v.obj2_start_pt_index, type);
	convert(old_ros_v.obj2_end_pt_index, ros_v.obj2_end_pt_index, type);
	convert(old_ros_v.slot_id, ros_v.slot_id, type);
	convert(old_ros_v.curb_exist, ros_v.curb_exist, type);
	convert(old_ros_v.slot_proc_slot_obj1_exist, ros_v.slot_proc_slot_obj1_exist, type);
	convert(old_ros_v.slot_proc_slot_obj2_exist, ros_v.slot_proc_slot_obj2_exist, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPerceptDebugInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	convert(old_ros_v.parking_slot_num, ros_v.parking_slot_num, type);
	convert(old_ros_v.uss_slot_list, ros_v.uss_slot_list, type);
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.uss_slot_param[i], ros_v.uss_slot_param[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.corner_points[i], ros_v.corner_points[i], type);
	}
}

