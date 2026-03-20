#pragma once

#include "base_convert.h"
#include "c/uss_perception_debug_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::AusspsUss &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.sita, ros_v.sita, type);
  convert(struct_v.slot_side, ros_v.slot_side, type);
  for (size_t i0 = 0; i0 < ros_v.border_type.size(); i0++) {
	  convert(struct_v.border_type[i0], ros_v.border_type[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.corner_point.size(); i1++) {
	  convert(struct_v.corner_point[i1], ros_v.corner_point[i1], type);
  }
  convert(struct_v.slot_type, ros_v.slot_type, type);
}

template <typename T2>
void convert(iflyauto::AusspsListUss &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.timestamp, ros_v.timestamp, type);
  convert(struct_v.parking_slot_size, ros_v.parking_slot_size, type);
  for (size_t i0 = 0; i0 < ros_v.parking_slots.size(); i0++) {
	  convert(struct_v.parking_slots[i0], ros_v.parking_slots[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SlotParameterType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.slot_type, ros_v.slot_type, type);
  convert(struct_v.slot_type_confirm, ros_v.slot_type_confirm, type);
  convert(struct_v.slot_start_index, ros_v.slot_start_index, type);
  convert(struct_v.slot_end_index, ros_v.slot_end_index, type);
  convert(struct_v.slot_cal_dis_car_to_obj1, ros_v.slot_cal_dis_car_to_obj1, type);
  convert(struct_v.slot_cal_dis_car_to_obj2, ros_v.slot_cal_dis_car_to_obj2, type);
  convert(struct_v.slot_length_detected1, ros_v.slot_length_detected1, type);
  convert(struct_v.slot_length_detected2, ros_v.slot_length_detected2, type);
  convert(struct_v.slot_length_detected3, ros_v.slot_length_detected3, type);
  convert(struct_v.slot_length_total, ros_v.slot_length_total, type);
  convert(struct_v.slot_depth_detected1, ros_v.slot_depth_detected1, type);
  convert(struct_v.slot_depth_detected2, ros_v.slot_depth_detected2, type);
  convert(struct_v.slot_depth_detected3, ros_v.slot_depth_detected3, type);
  convert(struct_v.slot_length, ros_v.slot_length, type);
  convert(struct_v.slot_depth_parallel, ros_v.slot_depth_parallel, type);
  convert(struct_v.slot_depth, ros_v.slot_depth, type);
  convert(struct_v.sub_slot_depth, ros_v.sub_slot_depth, type);
  convert(struct_v.dis_car_to_obj1_by_passing_slot, ros_v.dis_car_to_obj1_by_passing_slot, type);
  convert(struct_v.dis_car_to_obj2_by_passing_slot, ros_v.dis_car_to_obj2_by_passing_slot, type);
  convert(struct_v.car_pass_the_slot_end_pt_distance, ros_v.car_pass_the_slot_end_pt_distance, type);
  convert(struct_v.car_pass_the_slot_start_pt_distance, ros_v.car_pass_the_slot_start_pt_distance, type);
  convert(struct_v.apa_slot_detection_compensate_length_head, ros_v.apa_slot_detection_compensate_length_head, type);
  convert(struct_v.apa_slot_detection_compensate_length_tail, ros_v.apa_slot_detection_compensate_length_tail, type);
  convert(struct_v.obj1_width, ros_v.obj1_width, type);
  convert(struct_v.obj2_width, ros_v.obj2_width, type);
  convert(struct_v.obj1_width_confirm, ros_v.obj1_width_confirm, type);
  convert(struct_v.obj2_width_confirm, ros_v.obj2_width_confirm, type);
  convert(struct_v.obj1_type, ros_v.obj1_type, type);
  convert(struct_v.obj2_type, ros_v.obj2_type, type);
  convert(struct_v.obj1_start_pt_index, ros_v.obj1_start_pt_index, type);
  convert(struct_v.obj1_end_pt_index, ros_v.obj1_end_pt_index, type);
  convert(struct_v.obj2_start_pt_index, ros_v.obj2_start_pt_index, type);
  convert(struct_v.obj2_end_pt_index, ros_v.obj2_end_pt_index, type);
  convert(struct_v.slot_id, ros_v.slot_id, type);
  convert(struct_v.curb_exist, ros_v.curb_exist, type);
  convert(struct_v.slot_proc_slot_obj1_exist, ros_v.slot_proc_slot_obj1_exist, type);
  convert(struct_v.slot_proc_slot_obj2_exist, ros_v.slot_proc_slot_obj2_exist, type);
}

template <typename T2>
void convert(iflyauto::SlotInfoDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.slot_num, ros_v.slot_num, type);
  convert(struct_v.slot_confirm_seq, ros_v.slot_confirm_seq, type);
  convert(struct_v.slot_prev_id, ros_v.slot_prev_id, type);
  for (size_t i0 = 0; i0 < ros_v.slot_par.size(); i0++) {
	  convert(struct_v.slot_par[i0], ros_v.slot_par[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::CornerPoints &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.wr_index, ros_v.wr_index, type);
  convert(struct_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
  for (size_t i0 = 0; i0 < ros_v.obj_pt.size(); i0++) {
	  convert(struct_v.obj_pt[i0], ros_v.obj_pt[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.dis_from_car_to_obj.size(); i1++) {
	  convert(struct_v.dis_from_car_to_obj[i1], ros_v.dis_from_car_to_obj[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::UssPerceptDebugInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  convert(struct_v.parking_slot_num, ros_v.parking_slot_num, type);
  convert(struct_v.uss_slot_list, ros_v.uss_slot_list, type);
  for (size_t i0 = 0; i0 < ros_v.uss_slot_param.size(); i0++) {
	  convert(struct_v.uss_slot_param[i0], ros_v.uss_slot_param[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.corner_points.size(); i1++) {
	  convert(struct_v.corner_points[i1], ros_v.corner_points[i1], type);
  }
}

