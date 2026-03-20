#pragma once

#include "base_convert.h"
#include "c/hmi_service_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/hmi_mcu_out_c.h"
#include "struct_convert/hmi_soc_outer_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::PilotHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pilot_disp_acc_set_speed, ros_v.pilot_disp_acc_set_speed, type);
  convert(struct_v.pilot_disp_acc_set_headway, ros_v.pilot_disp_acc_set_headway, type);
  convert(struct_v.pilot_disp_acc_sts, ros_v.pilot_disp_acc_sts, type);
  convert(struct_v.pilot_disp_acc_notify, ros_v.pilot_disp_acc_notify, type);
  convert(struct_v.pilot_disp_scc_sts, ros_v.pilot_disp_scc_sts, type);
  convert(struct_v.pilot_disp_scc_notify, ros_v.pilot_disp_scc_notify, type);
  convert(struct_v.pilot_disp_noa_sts, ros_v.pilot_disp_noa_sts, type);
  convert(struct_v.pilot_disp_noa_notify, ros_v.pilot_disp_noa_notify, type);
  convert(struct_v.pilot_disp_adas_takeover_req, ros_v.pilot_disp_adas_takeover_req, type);
  convert(struct_v.pilot_disp_adas_takeover_reason, ros_v.pilot_disp_adas_takeover_reason, type);
  convert(struct_v.pilot_disp_lc_reason, ros_v.pilot_disp_lc_reason, type);
  convert(struct_v.pilot_disp_scc_handoff_warning, ros_v.pilot_disp_scc_handoff_warning, type);
  convert(struct_v.pilot_disp_dist_to_dest, ros_v.pilot_disp_dist_to_dest, type);
  convert(struct_v.pilot_disp_dist_to_station, ros_v.pilot_disp_dist_to_station, type);
}

template <typename T2>
void convert(iflyauto::ParkHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.park_disp_apa_status, ros_v.park_disp_apa_status, type);
  convert(struct_v.park_disp_apa_notify_req, ros_v.park_disp_apa_notify_req, type);
  convert(struct_v.park_disp_apa_mode_status, ros_v.park_disp_apa_mode_status, type);
  convert(struct_v.park_disp_rads_status, ros_v.park_disp_rads_status, type);
  convert(struct_v.park_disp_rads_notify_req, ros_v.park_disp_rads_notify_req, type);
  convert(struct_v.park_disp_nra_status, ros_v.park_disp_nra_status, type);
  convert(struct_v.park_disp_nra_notify, ros_v.park_disp_nra_notify, type);
  convert(struct_v.park_disp_pa_status, ros_v.park_disp_pa_status, type);
  convert(struct_v.park_disp_pa_notify, ros_v.park_disp_pa_notify, type);
  convert(struct_v.park_disp_hpp_status, ros_v.park_disp_hpp_status, type);
  convert(struct_v.park_disp_hpp_notify_req, ros_v.park_disp_hpp_notify_req, type);
  convert(struct_v.select_park_in_dir, ros_v.select_park_in_dir, type);
  convert(struct_v.apa_notify_remain_distance, ros_v.apa_notify_remain_distance, type);
  convert(struct_v.select_park_out_dir, ros_v.select_park_out_dir, type);
  convert(struct_v.planning_park_dir, ros_v.planning_park_dir, type);
  convert(struct_v.apa_mode_status, ros_v.apa_mode_status, type);
}

template <typename T2>
void convert(iflyauto::SwitchHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.noa_switch_response_display, ros_v.noa_switch_response_display, type);
  convert(struct_v.mnp_switch_response_display, ros_v.mnp_switch_response_display, type);
  convert(struct_v.apa_switch_response_display, ros_v.apa_switch_response_display, type);
  convert(struct_v.rpa_switch_response_display, ros_v.rpa_switch_response_display, type);
  convert(struct_v.hpp_switch_response_display, ros_v.hpp_switch_response_display, type);
  convert(struct_v.ldw_switch_response_display, ros_v.ldw_switch_response_display, type);
  convert(struct_v.ldp_switch_response_display, ros_v.ldp_switch_response_display, type);
  convert(struct_v.ldw_level_response_display, ros_v.ldw_level_response_display, type);
  convert(struct_v.aeb_switch_response_display, ros_v.aeb_switch_response_display, type);
  convert(struct_v.fcw_switch_response_display, ros_v.fcw_switch_response_display, type);
  convert(struct_v.fcw_level_response_display, ros_v.fcw_level_response_display, type);
  convert(struct_v.tsr_switch_response_display, ros_v.tsr_switch_response_display, type);
  convert(struct_v.elk_switch_response_display, ros_v.elk_switch_response_display, type);
  convert(struct_v.meb_switch_response_display, ros_v.meb_switch_response_display, type);
  convert(struct_v.amap_switch_response_display, ros_v.amap_switch_response_display, type);
  convert(struct_v.fcta_switch_response_display, ros_v.fcta_switch_response_display, type);
  convert(struct_v.fctb_switch_response_display, ros_v.fctb_switch_response_display, type);
  convert(struct_v.ihc_switch_response_display, ros_v.ihc_switch_response_display, type);
  convert(struct_v.dai_switch_response_display, ros_v.dai_switch_response_display, type);
  convert(struct_v.lcc_switch_response_display, ros_v.lcc_switch_response_display, type);
  convert(struct_v.bsd_switch_response_display, ros_v.bsd_switch_response_display, type);
  convert(struct_v.lca_switch_response_display, ros_v.lca_switch_response_display, type);
  convert(struct_v.dow_switch_response_display, ros_v.dow_switch_response_display, type);
  convert(struct_v.rcta_switch_response_display, ros_v.rcta_switch_response_display, type);
  convert(struct_v.rctb_switch_response_display, ros_v.rctb_switch_response_display, type);
  convert(struct_v.rcw_switch_response_display, ros_v.rcw_switch_response_display, type);
  convert(struct_v.dow_secondary_alert_switch_response_display, ros_v.dow_secondary_alert_switch_response_display, type);
  convert(struct_v.blue_light_switch_response_display, ros_v.blue_light_switch_response_display, type);
  convert(struct_v.function_degrade_switch_response_display, ros_v.function_degrade_switch_response_display, type);
}

template <typename T2>
void convert(iflyauto::AdasHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tsr_warning_display, ros_v.tsr_warning_display, type);
  convert(struct_v.tsr_speed_limit_display, ros_v.tsr_speed_limit_display, type);
  convert(struct_v.tsr_speed_unlimit_warning_display, ros_v.tsr_speed_unlimit_warning_display, type);
  convert(struct_v.tsr_state_display, ros_v.tsr_state_display, type);
  convert(struct_v.ldw_left_warning_display, ros_v.ldw_left_warning_display, type);
  convert(struct_v.ldw_right_warning_display, ros_v.ldw_right_warning_display, type);
  convert(struct_v.ldp_left_intervention_flag_display, ros_v.ldp_left_intervention_flag_display, type);
  convert(struct_v.ldp_right_intervention_flag_display, ros_v.ldp_right_intervention_flag_display, type);
  convert(struct_v.ldw_state_display, ros_v.ldw_state_display, type);
  convert(struct_v.ldp_state_display, ros_v.ldp_state_display, type);
  convert(struct_v.elk_left_intervention_flag_display, ros_v.elk_left_intervention_flag_display, type);
  convert(struct_v.elk_right_intervention_flag_display, ros_v.elk_right_intervention_flag_display, type);
  convert(struct_v.elk_state_display, ros_v.elk_state_display, type);
  convert(struct_v.meb_state_display, ros_v.meb_state_display, type);
  convert(struct_v.meb_request_direction_display, ros_v.meb_request_direction_display, type);
}

template <typename T2>
void convert(iflyauto::AccHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.offset_mode_display, ros_v.offset_mode_display, type);
  convert(struct_v.speed_offset_value_display, ros_v.speed_offset_value_display, type);
  convert(struct_v.speed_offset_percentage_display, ros_v.speed_offset_percentage_display, type);
  convert(struct_v.acc_spd_mode_ind_display, ros_v.acc_spd_mode_ind_display, type);
}

template <typename T2>
void convert(iflyauto::SccHmiDisPlay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.hands_off_detection_display, ros_v.hands_off_detection_display, type);
  convert(struct_v.traffic_light_stop_go_display, ros_v.traffic_light_stop_go_display, type);
  convert(struct_v.obstacle_bypass_display, ros_v.obstacle_bypass_display, type);
}

template <typename T2>
void convert(iflyauto::NoaHmiDisplay &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.noa_cruise_dclc_resp_display, ros_v.noa_cruise_dclc_resp_display, type);
  convert(struct_v.lane_change_style_display, ros_v.lane_change_style_display, type);
}

template <typename T2>
void convert(iflyauto::HmiDisplayInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pilot_hmi_display, ros_v.pilot_hmi_display, type);
  convert(struct_v.park_hmi_display, ros_v.park_hmi_display, type);
  convert(struct_v.hmi_switch_display, ros_v.hmi_switch_display, type);
  convert(struct_v.adas_hmi_display, ros_v.adas_hmi_display, type);
  convert(struct_v.acc_hmi_display, ros_v.acc_hmi_display, type);
  convert(struct_v.scc_hmi_display, ros_v.scc_hmi_display, type);
  convert(struct_v.noa_hmi_display, ros_v.noa_hmi_display, type);
}

template <typename T2>
void convert(iflyauto::HmiSetInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rads_in_display, ros_v.rads_in_display, type);
  convert(struct_v.pa_in_display, ros_v.pa_in_display, type);
  convert(struct_v.nra_in_display, ros_v.nra_in_display, type);
  convert(struct_v.apa_in_display, ros_v.apa_in_display, type);
  convert(struct_v.adas_in_display, ros_v.adas_in_display, type);
  convert(struct_v.pilot_in_display, ros_v.pilot_in_display, type);
  convert(struct_v.rpa_in_display, ros_v.rpa_in_display, type);
  convert(struct_v.hpp_in_display, ros_v.hpp_in_display, type);
  convert(struct_v.common_in, ros_v.common_in, type);
}

template <typename T2>
void convert(iflyauto::HmiMcuToSocBsw_CHERY_E0Y_MDC510 &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.active_btn_press, ros_v.active_btn_press, type);
  convert(struct_v.cancel_btn_press, ros_v.cancel_btn_press, type);
  convert(struct_v.vcu_actual_gear, ros_v.vcu_actual_gear, type);
  convert(struct_v.vcu_target_gear, ros_v.vcu_target_gear, type);
  convert(struct_v.cruise_speed_value, ros_v.cruise_speed_value, type);
  convert(struct_v.time_gap_value, ros_v.time_gap_value, type);
  convert(struct_v.break_btn_press, ros_v.break_btn_press, type);
  convert(struct_v.record_btn_press, ros_v.record_btn_press, type);
  convert(struct_v.adas_out_info, ros_v.adas_out_info, type);
  convert(struct_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
  convert(struct_v.rpa_info, ros_v.rpa_info, type);
  convert(struct_v.hmi_car_mode, ros_v.hmi_car_mode, type);
  convert(struct_v.auto_driving_btn_active, ros_v.auto_driving_btn_active, type);
  convert(struct_v.auto_driving_btn_cancel, ros_v.auto_driving_btn_cancel, type);
  convert(struct_v.cruise_speed_sync, ros_v.cruise_speed_sync, type);
  convert(struct_v.btn_reboot_adcc_req, ros_v.btn_reboot_adcc_req, type);
  convert(struct_v.reboot_adcc_standby, ros_v.reboot_adcc_standby, type);
  convert(struct_v.cruise_speed_cont, ros_v.cruise_speed_cont, type);
  convert(struct_v.hmi_set_info, ros_v.hmi_set_info, type);
  for (size_t i0 = 0; i0 < ros_v.reserved.size(); i0++) {
	  convert(struct_v.reserved[i0], ros_v.reserved[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::RPASoc2Mcu_CHERY_E0Y_MDC510 &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rpa_status, ros_v.rpa_status, type);
  convert(struct_v.rpa_parking_pause_ind, ros_v.rpa_parking_pause_ind, type);
  convert(struct_v.rpa_driver_operate_ind, ros_v.rpa_driver_operate_ind, type);
  convert(struct_v.rpa_quit_request_code, ros_v.rpa_quit_request_code, type);
  convert(struct_v.challenge1, ros_v.challenge1, type);
  convert(struct_v.challenge2, ros_v.challenge2, type);
  convert(struct_v.rpa_parking_active_sub_func_sts, ros_v.rpa_parking_active_sub_func_sts, type);
  convert(struct_v.ads_rpa_int_fun_sts, ros_v.ads_rpa_int_fun_sts, type);
  convert(struct_v.back_left_out_sts, ros_v.back_left_out_sts, type);
  convert(struct_v.back_right_out_sts, ros_v.back_right_out_sts, type);
  convert(struct_v.back_stright_out_sts, ros_v.back_stright_out_sts, type);
  convert(struct_v.front_left_out_sts, ros_v.front_left_out_sts, type);
  convert(struct_v.front_right_out_sts, ros_v.front_right_out_sts, type);
  convert(struct_v.front_stright_out_sts, ros_v.front_stright_out_sts, type);
  convert(struct_v.front_left_parallel_out_sts, ros_v.front_left_parallel_out_sts, type);
  convert(struct_v.front_right_parallel_out_sts, ros_v.front_right_parallel_out_sts, type);
  convert(struct_v.rpa_park_out_direction_st, ros_v.rpa_park_out_direction_st, type);
  convert(struct_v.parking_stop_dist, ros_v.parking_stop_dist, type);
  convert(struct_v.parking_complete_time, ros_v.parking_complete_time, type);
  convert(struct_v.rpa_out_fun_sts, ros_v.rpa_out_fun_sts, type);
  convert(struct_v.rpa_start_but_sts, ros_v.rpa_start_but_sts, type);
  convert(struct_v.prk_in_sts, ros_v.prk_in_sts, type);
}

template <typename T2>
void convert(iflyauto::HmiSocToMcuBsw_CHERY_E0Y_MDC510 &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.turn_light_req, ros_v.turn_light_req, type);
  convert(struct_v.beam_light_req, ros_v.beam_light_req, type);
  convert(struct_v.reboot_adcc_active, ros_v.reboot_adcc_active, type);
  convert(struct_v.icc_reboot_adcc_req, ros_v.icc_reboot_adcc_req, type);
  convert(struct_v.turn_light_state_machine, ros_v.turn_light_state_machine, type);
  convert(struct_v.hmi_display_info, ros_v.hmi_display_info, type);
  convert(struct_v.blue_light_req, ros_v.blue_light_req, type);
  convert(struct_v.hmi_parking_voice, ros_v.hmi_parking_voice, type);
  convert(struct_v.hmi_rap_info, ros_v.hmi_rap_info, type);
  convert(struct_v.door_lock_req, ros_v.door_lock_req, type);
  for (size_t i0 = 0; i0 < ros_v.reserved.size(); i0++) {
	  convert(struct_v.reserved[i0], ros_v.reserved[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::HmiSocToMcuBsw_CHERY_T26 &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.hmi_common, ros_v.hmi_common, type);
  convert(struct_v.hmi_acc_info, ros_v.hmi_acc_info, type);
  convert(struct_v.hmi_scc_info, ros_v.hmi_scc_info, type);
  convert(struct_v.hmi_noa_info, ros_v.hmi_noa_info, type);
  convert(struct_v.hmi_adas_info, ros_v.hmi_adas_info, type);
  convert(struct_v.calib_info, ros_v.calib_info, type);
  convert(struct_v.sensor_info, ros_v.sensor_info, type);
  for (size_t i0 = 0; i0 < ros_v.reserved.size(); i0++) {
	  convert(struct_v.reserved[i0], ros_v.reserved[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::HmiMcuToSocBsw_CHERY_T26 &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.active_btn_press, ros_v.active_btn_press, type);
  convert(struct_v.cancel_btn_press, ros_v.cancel_btn_press, type);
  convert(struct_v.vcu_actual_gear, ros_v.vcu_actual_gear, type);
  convert(struct_v.vcu_target_gear, ros_v.vcu_target_gear, type);
  convert(struct_v.cruise_speed_value, ros_v.cruise_speed_value, type);
  convert(struct_v.time_gap_value, ros_v.time_gap_value, type);
  convert(struct_v.break_btn_press, ros_v.break_btn_press, type);
  convert(struct_v.record_btn_press, ros_v.record_btn_press, type);
  convert(struct_v.adas_out_info, ros_v.adas_out_info, type);
  convert(struct_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
  convert(struct_v.adas_in, ros_v.adas_in, type);
  convert(struct_v.pilot_in, ros_v.pilot_in, type);
  convert(struct_v.calib_active_switch, ros_v.calib_active_switch, type);
  for (size_t i0 = 0; i0 < ros_v.reserved.size(); i0++) {
	  convert(struct_v.reserved[i0], ros_v.reserved[i0], type);
  }
}

