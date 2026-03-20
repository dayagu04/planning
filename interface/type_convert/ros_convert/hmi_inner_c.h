#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ADASIn.h"
#include "struct_msgs_v2_10/ADASIn.h"
#include "struct_msgs/APAIn.h"
#include "struct_msgs_v2_10/APAIn.h"
#include "struct_msgs/ApaFreeSlotInfo.h"
#include "struct_msgs_v2_10/ApaFreeSlotInfo.h"
#include "struct_msgs/ApaUserPreference.h"
#include "struct_msgs_v2_10/ApaUserPreference.h"
#include "struct_msgs/EHPIn.h"
#include "struct_msgs_v2_10/EHPIn.h"
#include "struct_msgs/EhpParam.h"
#include "struct_msgs_v2_10/EhpParam.h"
#include "struct_msgs/HPPIn.h"
#include "struct_msgs_v2_10/HPPIn.h"
#include "struct_msgs/HmiInner.h"
#include "struct_msgs_v2_10/HmiInner.h"
#include "struct_msgs/PilotIn.h"
#include "struct_msgs_v2_10/PilotIn.h"
#include "struct_msgs/PilotUserPreference.h"
#include "struct_msgs_v2_10/PilotUserPreference.h"
#include "struct_msgs/RADSIn.h"
#include "struct_msgs_v2_10/RADSIn.h"
#include "struct_msgs/RPAIn.h"
#include "struct_msgs_v2_10/RPAIn.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ADASIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fcw_set_sensitivity_level, ros_v.fcw_set_sensitivity_level, type);
	convert(old_ros_v.ldw_set_sensitivity_level, ros_v.ldw_set_sensitivity_level, type);
	convert(old_ros_v.fcw_main_switch, ros_v.fcw_main_switch, type);
	convert(old_ros_v.aeb_main_switch, ros_v.aeb_main_switch, type);
	convert(old_ros_v.meb_main_switch, ros_v.meb_main_switch, type);
	convert(old_ros_v.tsr_main_switch, ros_v.tsr_main_switch, type);
	convert(old_ros_v.ihc_main_switch, ros_v.ihc_main_switch, type);
	convert(old_ros_v.ldw_main_switch, ros_v.ldw_main_switch, type);
	convert(old_ros_v.elk_main_switch, ros_v.elk_main_switch, type);
	convert(old_ros_v.bsd_main_switch, ros_v.bsd_main_switch, type);
	convert(old_ros_v.lca_main_switch, ros_v.lca_main_switch, type);
	convert(old_ros_v.dow_main_switch, ros_v.dow_main_switch, type);
	convert(old_ros_v.fcta_main_switch, ros_v.fcta_main_switch, type);
	convert(old_ros_v.rcta_main_switch, ros_v.rcta_main_switch, type);
	convert(old_ros_v.rcw_main_switch, ros_v.rcw_main_switch, type);
	convert(old_ros_v.ldp_main_switch, ros_v.ldp_main_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::APAIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.apa_active_switch, ros_v.apa_active_switch, type);
	convert(old_ros_v.apa_work_mode, ros_v.apa_work_mode, type);
	convert(old_ros_v.apa_cancel_switch, ros_v.apa_cancel_switch, type);
	convert(old_ros_v.apa_select_slot_id, ros_v.apa_select_slot_id, type);
	convert(old_ros_v.apa_start, ros_v.apa_start, type);
	convert(old_ros_v.apa_resume, ros_v.apa_resume, type);
	convert(old_ros_v.apa_parking_direction, ros_v.apa_parking_direction, type);
	convert(old_ros_v.apa_park_out_direction, ros_v.apa_park_out_direction, type);
	convert(old_ros_v.apa_user_preference, ros_v.apa_user_preference, type);
	convert(old_ros_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
	convert(old_ros_v.stop_parking, ros_v.stop_parking, type);
	convert(old_ros_v.outside_parking, ros_v.outside_parking, type);
	convert(old_ros_v.change_slot, ros_v.change_slot, type);
	convert(old_ros_v.apa_main_switch, ros_v.apa_main_switch, type);
	convert(old_ros_v.apa_avm_main_switch, ros_v.apa_avm_main_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ApaFreeSlotInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.type, ros_v.type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.corner_points[i], ros_v.corner_points[i], type);
	}
	convert(old_ros_v.is_free_slot_selected, ros_v.is_free_slot_selected, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ApaUserPreference &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.select_source, ros_v.select_source, type);
	convert(old_ros_v.horizontal_preference_slot, ros_v.horizontal_preference_slot, type);
	convert(old_ros_v.vertical_preference_slot, ros_v.vertical_preference_slot, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EHPIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.action, ros_v.action, type);
	convert(old_ros_v.param, ros_v.param, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EhpParam &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.map_file_id, ros_v.map_file_id, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.filename[i], ros_v.filename[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HPPIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.hpp_cancel_switch, ros_v.hpp_cancel_switch, type);
	convert(old_ros_v.hpp_active_switch, ros_v.hpp_active_switch, type);
	convert(old_ros_v.start_memory_parking, ros_v.start_memory_parking, type);
	convert(old_ros_v.continue_memory_parking, ros_v.continue_memory_parking, type);
	convert(old_ros_v.stop_memory_parking, ros_v.stop_memory_parking, type);
	convert(old_ros_v.start_route_learning, ros_v.start_route_learning, type);
	convert(old_ros_v.complete_route_learning, ros_v.complete_route_learning, type);
	convert(old_ros_v.stop_route_learning, ros_v.stop_route_learning, type);
	convert(old_ros_v.resume_location, ros_v.resume_location, type);
	convert(old_ros_v.save_route, ros_v.save_route, type);
	convert(old_ros_v.cancel_route, ros_v.cancel_route, type);
	convert(old_ros_v.cruise_mode_switch, ros_v.cruise_mode_switch, type);
	convert(old_ros_v.hpp_main_switch, ros_v.hpp_main_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiInner &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.apa_in, ros_v.apa_in, type);
	convert(old_ros_v.rpa_in, ros_v.rpa_in, type);
	convert(old_ros_v.hpp_in, ros_v.hpp_in, type);
	convert(old_ros_v.ehp_in, ros_v.ehp_in, type);
	convert(old_ros_v.adas_in, ros_v.adas_in, type);
	convert(old_ros_v.pilot_in, ros_v.pilot_in, type);
	convert(old_ros_v.calib_active_switch, ros_v.calib_active_switch, type);
	convert(old_ros_v.data_record_switch, ros_v.data_record_switch, type);
	convert(old_ros_v.rads_in, ros_v.rads_in, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PilotIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.acc_active_switch, ros_v.acc_active_switch, type);
	convert(old_ros_v.acc_cancel_switch, ros_v.acc_cancel_switch, type);
	convert(old_ros_v.acc_set_disp_speed, ros_v.acc_set_disp_speed, type);
	convert(old_ros_v.acc_set_spd_pm, ros_v.acc_set_spd_pm, type);
	convert(old_ros_v.acc_set_time_gap, ros_v.acc_set_time_gap, type);
	convert(old_ros_v.acc_set_time_interval, ros_v.acc_set_time_interval, type);
	convert(old_ros_v.acc_stop2go, ros_v.acc_stop2go, type);
	convert(old_ros_v.scc_active_switch, ros_v.scc_active_switch, type);
	convert(old_ros_v.scc_cancel_switch, ros_v.scc_cancel_switch, type);
	convert(old_ros_v.noa_active_switch, ros_v.noa_active_switch, type);
	convert(old_ros_v.noa_voice_promp_switch, ros_v.noa_voice_promp_switch, type);
	convert(old_ros_v.noa_cancel_switch, ros_v.noa_cancel_switch, type);
	convert(old_ros_v.pilot_user_preference, ros_v.pilot_user_preference, type);
	convert(old_ros_v.quick_speed_set, ros_v.quick_speed_set, type);
	convert(old_ros_v.acc_main_switch, ros_v.acc_main_switch, type);
	convert(old_ros_v.scc_main_switch, ros_v.scc_main_switch, type);
	convert(old_ros_v.noa_main_switch, ros_v.noa_main_switch, type);
	convert(old_ros_v.noa_cruise_dclc_switch, ros_v.noa_cruise_dclc_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PilotUserPreference &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.select_source, ros_v.select_source, type);
	convert(old_ros_v.preference_line, ros_v.preference_line, type);
	convert(old_ros_v.drive_mode, ros_v.drive_mode, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RADSIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rads_cancel_switch, ros_v.rads_cancel_switch, type);
	convert(old_ros_v.rads_active_switch, ros_v.rads_active_switch, type);
	convert(old_ros_v.rads_start_switch, ros_v.rads_start_switch, type);
	convert(old_ros_v.rads_main_switch, ros_v.rads_main_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RPAIn &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rpa_device_fail_sts, ros_v.rpa_device_fail_sts, type);
	convert(old_ros_v.rpa_mod_req, ros_v.rpa_mod_req, type);
	convert(old_ros_v.rpa_mod_select, ros_v.rpa_mod_select, type);
	convert(old_ros_v.rpa_btn_sts, ros_v.rpa_btn_sts, type);
	convert(old_ros_v.straight_in_btn_sts, ros_v.straight_in_btn_sts, type);
	convert(old_ros_v.straight_out_btn_sts, ros_v.straight_out_btn_sts, type);
	convert(old_ros_v.rpa_btn_cancel_sts, ros_v.rpa_btn_cancel_sts, type);
	convert(old_ros_v.prk_out_mod_sel_rmt, ros_v.prk_out_mod_sel_rmt, type);
	convert(old_ros_v.select_slot_id, ros_v.select_slot_id, type);
	convert(old_ros_v.heart_beat_signal, ros_v.heart_beat_signal, type);
	convert(old_ros_v.rpa_device_dis_sts, ros_v.rpa_device_dis_sts, type);
	convert(old_ros_v.ble_conn_sts, ros_v.ble_conn_sts, type);
	convert(old_ros_v.ble_err_status, ros_v.ble_err_status, type);
	convert(old_ros_v.addc_fusa_sts, ros_v.addc_fusa_sts, type);
}

REG_CONVERT_SINGLE(_iflytek_hmi_inner_converter, "/iflytek/hmi/inner", HmiInner);
