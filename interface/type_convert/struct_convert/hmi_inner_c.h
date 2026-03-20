#pragma once

#include "base_convert.h"
#include "c/hmi_inner_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_convert/camera_perception_parking_slot_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ApaUserPreference &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.select_source, ros_v.select_source, type);
  convert(struct_v.horizontal_preference_slot, ros_v.horizontal_preference_slot, type);
  convert(struct_v.vertical_preference_slot, ros_v.vertical_preference_slot, type);
}

template <typename T2>
void convert(iflyauto::ApaFreeSlotInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_free_slot_selected, ros_v.is_free_slot_selected, type);
  convert(struct_v.free_slot_activate, ros_v.free_slot_activate, type);
  convert(struct_v.type, ros_v.type, type);
  for (size_t i0 = 0; i0 < ros_v.corner_points.size(); i0++) {
	  convert(struct_v.corner_points[i0], ros_v.corner_points[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.collision_dete.size(); i1++) {
	  convert(struct_v.collision_dete[i1], ros_v.collision_dete[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::APAIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_main_switch, ros_v.apa_main_switch, type);
  convert(struct_v.apa_active_switch, ros_v.apa_active_switch, type);
  convert(struct_v.apa_work_mode, ros_v.apa_work_mode, type);
  convert(struct_v.apa_cancel_switch, ros_v.apa_cancel_switch, type);
  convert(struct_v.apa_select_slot_id, ros_v.apa_select_slot_id, type);
  convert(struct_v.apa_start, ros_v.apa_start, type);
  convert(struct_v.apa_resume, ros_v.apa_resume, type);
  convert(struct_v.apa_parking_direction, ros_v.apa_parking_direction, type);
  convert(struct_v.apa_park_out_direction, ros_v.apa_park_out_direction, type);
  convert(struct_v.apa_avm_main_switch, ros_v.apa_avm_main_switch, type);
  convert(struct_v.apa_user_preference, ros_v.apa_user_preference, type);
  convert(struct_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
  convert(struct_v.stop_parking, ros_v.stop_parking, type);
  convert(struct_v.outside_parking, ros_v.outside_parking, type);
  convert(struct_v.change_slot, ros_v.change_slot, type);
  convert(struct_v.sapa_start, ros_v.sapa_start, type);
  convert(struct_v.sapa_cancel, ros_v.sapa_cancel, type);
  convert(struct_v.apa_park_out_active_switch, ros_v.apa_park_out_active_switch, type);
  convert(struct_v.switch_parking_direction, ros_v.switch_parking_direction, type);
  convert(struct_v.parking_speed_set, ros_v.parking_speed_set, type);
}

template <typename T2>
void convert(iflyauto::RPAIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.bncm_chanllenge_result, ros_v.bncm_chanllenge_result, type);
  convert(struct_v.bncm_heartbeat_result, ros_v.bncm_heartbeat_result, type);
  convert(struct_v.rpa_device_fail_sts, ros_v.rpa_device_fail_sts, type);
  convert(struct_v.rpa_mod_req, ros_v.rpa_mod_req, type);
  convert(struct_v.rpa_mod_select, ros_v.rpa_mod_select, type);
  convert(struct_v.rpa_btn_sts, ros_v.rpa_btn_sts, type);
  convert(struct_v.straight_in_btn_sts, ros_v.straight_in_btn_sts, type);
  convert(struct_v.straight_out_btn_sts, ros_v.straight_out_btn_sts, type);
  convert(struct_v.rpa_btn_cancel_sts, ros_v.rpa_btn_cancel_sts, type);
  convert(struct_v.prk_out_mod_sel_rmt, ros_v.prk_out_mod_sel_rmt, type);
  convert(struct_v.select_slot_id, ros_v.select_slot_id, type);
  convert(struct_v.heart_beat_signal, ros_v.heart_beat_signal, type);
  convert(struct_v.rpa_device_dis_sts, ros_v.rpa_device_dis_sts, type);
  convert(struct_v.ble_conn_sts, ros_v.ble_conn_sts, type);
  convert(struct_v.ble_err_status, ros_v.ble_err_status, type);
  convert(struct_v.addc_fusa_sts, ros_v.addc_fusa_sts, type);
  convert(struct_v.rpa_main_switch, ros_v.rpa_main_switch, type);
  convert(struct_v.rpa_pause_switch, ros_v.rpa_pause_switch, type);
  convert(struct_v.rpa_resume_switch, ros_v.rpa_resume_switch, type);
  convert(struct_v.rpa_switch_req, ros_v.rpa_switch_req, type);
}

template <typename T2>
void convert(iflyauto::HPPIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.hpp_main_switch, ros_v.hpp_main_switch, type);
  convert(struct_v.hpp_cancel_switch, ros_v.hpp_cancel_switch, type);
  convert(struct_v.hpp_active_switch, ros_v.hpp_active_switch, type);
  convert(struct_v.start_memory_parking, ros_v.start_memory_parking, type);
  convert(struct_v.continue_memory_parking, ros_v.continue_memory_parking, type);
  convert(struct_v.stop_memory_parking, ros_v.stop_memory_parking, type);
  convert(struct_v.start_route_learning, ros_v.start_route_learning, type);
  convert(struct_v.complete_route_learning, ros_v.complete_route_learning, type);
  convert(struct_v.stop_route_learning, ros_v.stop_route_learning, type);
  convert(struct_v.resume_location, ros_v.resume_location, type);
  convert(struct_v.save_route, ros_v.save_route, type);
  convert(struct_v.cancel_route, ros_v.cancel_route, type);
  convert(struct_v.cruise_mode_switch, ros_v.cruise_mode_switch, type);
  convert(struct_v.need_gnss_loss_signal, ros_v.need_gnss_loss_signal, type);
  convert(struct_v.target_prk_space_id, ros_v.target_prk_space_id, type);
  convert(struct_v.try_it_now_request, ros_v.try_it_now_request, type);
}

template <typename T2>
void convert(iflyauto::EhpParam &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.map_file_id, ros_v.map_file_id, type);
  for (size_t i0 = 0; i0 < ros_v.filename.size(); i0++) {
	  convert(struct_v.filename[i0], ros_v.filename[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::EHPIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.action, ros_v.action, type);
  convert(struct_v.param, ros_v.param, type);
}

template <typename T2>
void convert(iflyauto::ADASIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fcw_main_switch, ros_v.fcw_main_switch, type);
  convert(struct_v.fcw_set_sensitivity_level, ros_v.fcw_set_sensitivity_level, type);
  convert(struct_v.aeb_main_switch, ros_v.aeb_main_switch, type);
  convert(struct_v.meb_main_switch, ros_v.meb_main_switch, type);
  convert(struct_v.tsr_main_switch, ros_v.tsr_main_switch, type);
  convert(struct_v.ihc_main_switch, ros_v.ihc_main_switch, type);
  convert(struct_v.ldw_main_switch, ros_v.ldw_main_switch, type);
  convert(struct_v.ldw_set_sensitivity_level, ros_v.ldw_set_sensitivity_level, type);
  convert(struct_v.elk_main_switch, ros_v.elk_main_switch, type);
  convert(struct_v.bsd_main_switch, ros_v.bsd_main_switch, type);
  convert(struct_v.lca_main_switch, ros_v.lca_main_switch, type);
  convert(struct_v.dow_main_switch, ros_v.dow_main_switch, type);
  convert(struct_v.fcta_main_switch, ros_v.fcta_main_switch, type);
  convert(struct_v.fctb_main_switch, ros_v.fctb_main_switch, type);
  convert(struct_v.rcta_main_switch, ros_v.rcta_main_switch, type);
  convert(struct_v.rctb_main_switch, ros_v.rctb_main_switch, type);
  convert(struct_v.rcw_main_switch, ros_v.rcw_main_switch, type);
  convert(struct_v.ldp_main_switch, ros_v.ldp_main_switch, type);
  convert(struct_v.amap_main_switch, ros_v.amap_main_switch, type);
  convert(struct_v.dai_main_switch, ros_v.dai_main_switch, type);
  convert(struct_v.dow_secondary_alert_main_switch, ros_v.dow_secondary_alert_main_switch, type);
}

template <typename T2>
void convert(iflyauto::PilotUserPreference &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.select_source, ros_v.select_source, type);
  convert(struct_v.preference_line, ros_v.preference_line, type);
  convert(struct_v.drive_mode, ros_v.drive_mode, type);
}

template <typename T2>
void convert(iflyauto::SpeedOffset &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.offset_mode, ros_v.offset_mode, type);
  convert(struct_v.speed_offset_set, ros_v.speed_offset_set, type);
  convert(struct_v.speed_offset_value, ros_v.speed_offset_value, type);
}

template <typename T2>
void convert(iflyauto::PilotIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acc_main_switch, ros_v.acc_main_switch, type);
  convert(struct_v.acc_active_switch, ros_v.acc_active_switch, type);
  convert(struct_v.acc_cancel_switch, ros_v.acc_cancel_switch, type);
  convert(struct_v.acc_set_disp_speed, ros_v.acc_set_disp_speed, type);
  convert(struct_v.acc_set_spd_pm, ros_v.acc_set_spd_pm, type);
  convert(struct_v.acc_set_time_gap, ros_v.acc_set_time_gap, type);
  convert(struct_v.acc_set_time_interval, ros_v.acc_set_time_interval, type);
  convert(struct_v.acc_stop2go, ros_v.acc_stop2go, type);
  convert(struct_v.scc_main_switch, ros_v.scc_main_switch, type);
  convert(struct_v.scc_active_switch, ros_v.scc_active_switch, type);
  convert(struct_v.scc_cancel_switch, ros_v.scc_cancel_switch, type);
  convert(struct_v.noa_main_switch, ros_v.noa_main_switch, type);
  convert(struct_v.noa_active_switch, ros_v.noa_active_switch, type);
  convert(struct_v.noa_downgrade_scc_switch, ros_v.noa_downgrade_scc_switch, type);
  convert(struct_v.scc_upgrade_noa_switch, ros_v.scc_upgrade_noa_switch, type);
  convert(struct_v.noa_voice_promp_switch, ros_v.noa_voice_promp_switch, type);
  convert(struct_v.noa_cruise_dclc_switch, ros_v.noa_cruise_dclc_switch, type);
  convert(struct_v.mnp_main_switch, ros_v.mnp_main_switch, type);
  convert(struct_v.noa_cancel_switch, ros_v.noa_cancel_switch, type);
  convert(struct_v.pilot_user_preference, ros_v.pilot_user_preference, type);
  convert(struct_v.quick_speed_set, ros_v.quick_speed_set, type);
  convert(struct_v.speed_offset, ros_v.speed_offset, type);
  convert(struct_v.hands_off_detection, ros_v.hands_off_detection, type);
  convert(struct_v.lane_change_style, ros_v.lane_change_style, type);
  convert(struct_v.traffic_light_stop_go, ros_v.traffic_light_stop_go, type);
  convert(struct_v.ObstacleBypass, ros_v.ObstacleBypass, type);
  convert(struct_v.voice_turn_switch, ros_v.voice_turn_switch, type);
  convert(struct_v.acc_spd_mode_set, ros_v.acc_spd_mode_set, type);
}

template <typename T2>
void convert(iflyauto::RADSIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rads_main_switch, ros_v.rads_main_switch, type);
  convert(struct_v.rads_cancel_switch, ros_v.rads_cancel_switch, type);
  convert(struct_v.rads_active_switch, ros_v.rads_active_switch, type);
  convert(struct_v.rads_start_switch, ros_v.rads_start_switch, type);
}

template <typename T2>
void convert(iflyauto::PAIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pa_main_switch, ros_v.pa_main_switch, type);
  convert(struct_v.pa_cancel_switch, ros_v.pa_cancel_switch, type);
  convert(struct_v.pa_active_switch, ros_v.pa_active_switch, type);
  convert(struct_v.pa_start_switch, ros_v.pa_start_switch, type);
  convert(struct_v.pa_direction, ros_v.pa_direction, type);
}

template <typename T2>
void convert(iflyauto::NRAIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.nra_main_switch, ros_v.nra_main_switch, type);
  convert(struct_v.nra_cancel_switch, ros_v.nra_cancel_switch, type);
  convert(struct_v.nra_active_switch, ros_v.nra_active_switch, type);
  convert(struct_v.nra_start_switch, ros_v.nra_start_switch, type);
  convert(struct_v.nra_resume_switch, ros_v.nra_resume_switch, type);
}

template <typename T2>
void convert(iflyauto::CommonIn &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.blue_light_main_switch, ros_v.blue_light_main_switch, type);
}

template <typename T2>
void convert(iflyauto::HmiInner &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.is_valid, ros_v.is_valid, type);
  convert(struct_v.apa_in, ros_v.apa_in, type);
  convert(struct_v.rpa_in, ros_v.rpa_in, type);
  convert(struct_v.hpp_in, ros_v.hpp_in, type);
  convert(struct_v.ehp_in, ros_v.ehp_in, type);
  convert(struct_v.adas_in, ros_v.adas_in, type);
  convert(struct_v.pilot_in, ros_v.pilot_in, type);
  convert(struct_v.calib_active_switch, ros_v.calib_active_switch, type);
  convert(struct_v.data_record_switch, ros_v.data_record_switch, type);
  convert(struct_v.rads_in, ros_v.rads_in, type);
  convert(struct_v.pa_in, ros_v.pa_in, type);
  convert(struct_v.nra_in, ros_v.nra_in, type);
  convert(struct_v.common_in, ros_v.common_in, type);
  convert(struct_v.data_upload_btn, ros_v.data_upload_btn, type);
  convert(struct_v.record_upload_btn, ros_v.record_upload_btn, type);
  convert(struct_v.sr_switch, ros_v.sr_switch, type);
}

