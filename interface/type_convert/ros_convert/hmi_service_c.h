#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/HmiMcuToSocBsw_CHERY_E0Y_MDC510.h"
#include "struct_msgs_v2_10/HmiMcuToSocBsw_CHERY_E0Y_MDC510.h"
#include "struct_msgs/HmiMcuToSocBsw_CHERY_T26.h"
#include "struct_msgs_v2_10/HmiMcuToSocBsw_CHERY_T26.h"
#include "struct_msgs/HmiSocToMcuBsw_CHERY_E0Y_MDC510.h"
#include "struct_msgs_v2_10/HmiSocToMcuBsw_CHERY_E0Y_MDC510.h"
#include "struct_msgs/HmiSocToMcuBsw_CHERY_T26.h"
#include "struct_msgs_v2_10/HmiSocToMcuBsw_CHERY_T26.h"
#include "struct_msgs/RPASoc2Mcu_CHERY_E0Y_MDC510.h"
#include "struct_msgs_v2_10/RPASoc2Mcu_CHERY_E0Y_MDC510.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiMcuToSocBsw_CHERY_E0Y_MDC510 &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.active_btn_press, ros_v.active_btn_press, type);
	convert(old_ros_v.cancel_btn_press, ros_v.cancel_btn_press, type);
	convert(old_ros_v.vcu_actual_gear, ros_v.vcu_actual_gear, type);
	convert(old_ros_v.vcu_target_gear, ros_v.vcu_target_gear, type);
	convert(old_ros_v.cruise_speed_value, ros_v.cruise_speed_value, type);
	convert(old_ros_v.time_gap_value, ros_v.time_gap_value, type);
	convert(old_ros_v.break_btn_press, ros_v.break_btn_press, type);
	convert(old_ros_v.record_btn_press, ros_v.record_btn_press, type);
	convert(old_ros_v.adas_out_info, ros_v.adas_out_info, type);
	convert(old_ros_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
	convert(old_ros_v.rpa_info, ros_v.rpa_info, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.reserved[i], ros_v.reserved[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiMcuToSocBsw_CHERY_T26 &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.active_btn_press, ros_v.active_btn_press, type);
	convert(old_ros_v.cancel_btn_press, ros_v.cancel_btn_press, type);
	convert(old_ros_v.vcu_actual_gear, ros_v.vcu_actual_gear, type);
	convert(old_ros_v.vcu_target_gear, ros_v.vcu_target_gear, type);
	convert(old_ros_v.cruise_speed_value, ros_v.cruise_speed_value, type);
	convert(old_ros_v.time_gap_value, ros_v.time_gap_value, type);
	convert(old_ros_v.break_btn_press, ros_v.break_btn_press, type);
	convert(old_ros_v.record_btn_press, ros_v.record_btn_press, type);
	convert(old_ros_v.adas_out_info, ros_v.adas_out_info, type);
	convert(old_ros_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
	convert(old_ros_v.adas_in, ros_v.adas_in, type);
	convert(old_ros_v.pilot_in, ros_v.pilot_in, type);
	convert(old_ros_v.calib_active_switch, ros_v.calib_active_switch, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.reserved[i], ros_v.reserved[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSocToMcuBsw_CHERY_E0Y_MDC510 &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.rpa_soc_to_mcu, ros_v.rpa_soc_to_mcu, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.reserved[i], ros_v.reserved[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSocToMcuBsw_CHERY_T26 &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.hmi_common, ros_v.hmi_common, type);
	convert(old_ros_v.hmi_acc_info, ros_v.hmi_acc_info, type);
	convert(old_ros_v.hmi_scc_info, ros_v.hmi_scc_info, type);
	convert(old_ros_v.hmi_noa_info, ros_v.hmi_noa_info, type);
	convert(old_ros_v.hmi_adas_info, ros_v.hmi_adas_info, type);
	convert(old_ros_v.calib_info, ros_v.calib_info, type);
	convert(old_ros_v.sensor_info, ros_v.sensor_info, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.reserved[i], ros_v.reserved[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RPASoc2Mcu_CHERY_E0Y_MDC510 &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rpa_status, ros_v.rpa_status, type);
	convert(old_ros_v.rpa_parking_pause_ind, ros_v.rpa_parking_pause_ind, type);
	convert(old_ros_v.rpa_driver_operate_ind, ros_v.rpa_driver_operate_ind, type);
	convert(old_ros_v.rpa_quit_request_code, ros_v.rpa_quit_request_code, type);
	convert(old_ros_v.challenge1, ros_v.challenge1, type);
	convert(old_ros_v.challenge2, ros_v.challenge2, type);
	convert(old_ros_v.rpa_parking_active_sub_func_sts, ros_v.rpa_parking_active_sub_func_sts, type);
	convert(old_ros_v.ads_rpa_int_fun_sts, ros_v.ads_rpa_int_fun_sts, type);
	convert(old_ros_v.back_left_out_sts, ros_v.back_left_out_sts, type);
	convert(old_ros_v.back_right_out_sts, ros_v.back_right_out_sts, type);
	convert(old_ros_v.back_stright_out_sts, ros_v.back_stright_out_sts, type);
	convert(old_ros_v.front_left_out_sts, ros_v.front_left_out_sts, type);
	convert(old_ros_v.front_right_out_sts, ros_v.front_right_out_sts, type);
	convert(old_ros_v.front_stright_out_sts, ros_v.front_stright_out_sts, type);
	convert(old_ros_v.front_left_parallel_out_sts, ros_v.front_left_parallel_out_sts, type);
	convert(old_ros_v.front_right_parallel_out_sts, ros_v.front_right_parallel_out_sts, type);
	convert(old_ros_v.rpa_park_out_direction_st, ros_v.rpa_park_out_direction_st, type);
	convert(old_ros_v.parking_stop_dist, ros_v.parking_stop_dist, type);
	convert(old_ros_v.parking_complete_time, ros_v.parking_complete_time, type);
	convert(old_ros_v.rpa_out_fun_sts, ros_v.rpa_out_fun_sts, type);
	convert(old_ros_v.rpa_start_but_sts, ros_v.rpa_start_but_sts, type);
	convert(old_ros_v.prk_in_sts, ros_v.prk_in_sts, type);
}

REG_CONVERT_SINGLE(_iflytek_hmi_mcu_to_soc_bsw_chery_e0y_mdc510_converter, "/iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510", HmiMcuToSocBsw_CHERY_E0Y_MDC510);
REG_CONVERT_SINGLE(_iflytek_hmi_mcu_to_soc_bsw_chery_t26_fdc_converter, "/iflytek/hmi/mcu_to_soc_bsw/chery_t26_fdc", HmiMcuToSocBsw_CHERY_T26);
REG_CONVERT_SINGLE(_iflytek_hmi_soc_to_mcu_bsw_chery_e0y_mdc510_converter, "/iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510", HmiSocToMcuBsw_CHERY_E0Y_MDC510);
REG_CONVERT_SINGLE(_iflytek_hmi_soc_to_mcu_bsw_chery_t26_fdc_converter, "/iflytek/hmi/soc_to_mcu_bsw/chery_t26_fdc", HmiSocToMcuBsw_CHERY_T26);
