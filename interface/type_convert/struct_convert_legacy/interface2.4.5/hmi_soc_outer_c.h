#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/hmi_soc_outer_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiCommon &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_sharp_turn, ros_v.is_sharp_turn, type);
  convert(struct_v.is_weather_abnormal, ros_v.is_weather_abnormal, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiNoaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.noa_odometer_info, ros_v.noa_odometer_info, type);
  convert(struct_v.noa_total_odometer_info, ros_v.noa_total_odometer_info, type);
  convert(struct_v.noa_hour, ros_v.noa_hour, type);
  convert(struct_v.noa_minute, ros_v.noa_minute, type);
  convert(struct_v.noa_maxspd, ros_v.noa_maxspd, type);
  convert(struct_v.noa_average_spd, ros_v.noa_average_spd, type);
  convert(struct_v.noa_state, ros_v.noa_state, type);
  convert(struct_v.noa_not_satisfied_condition, ros_v.noa_not_satisfied_condition, type);
  convert(struct_v.noa_activate_resp, ros_v.noa_activate_resp, type);
  convert(struct_v.noa_voice_remind, ros_v.noa_voice_remind, type);
  convert(struct_v.noa_adu_error, ros_v.noa_adu_error, type);
  convert(struct_v.noa_takeover_req_lv, ros_v.noa_takeover_req_lv, type);
  convert(struct_v.noa_tor_tips, ros_v.noa_tor_tips, type);
  convert(struct_v.noa_changelane_function_forbidden, ros_v.noa_changelane_function_forbidden, type);
  convert(struct_v.noa_override_mode, ros_v.noa_override_mode, type);
  convert(struct_v.noa_voice_prompt_set_resp, ros_v.noa_voice_prompt_set_resp, type);
  convert(struct_v.noa_cruise_dclc_swset_resp, ros_v.noa_cruise_dclc_swset_resp, type);
  convert(struct_v.noa_safe_lanchg_remind, ros_v.noa_safe_lanchg_remind, type);
  convert(struct_v.noa_close_door_req, ros_v.noa_close_door_req, type);
  convert(struct_v.noa_belted_req, ros_v.noa_belted_req, type);
  convert(struct_v.noa_brakeing_req, ros_v.noa_brakeing_req, type);
  convert(struct_v.scc_in_out_remind, ros_v.scc_in_out_remind, type);
  convert(struct_v.distance_to_destination, ros_v.distance_to_destination, type);
  convert(struct_v.distance_to_amp, ros_v.distance_to_amp, type);
  convert(struct_v.distance_to_tunnel, ros_v.distance_to_tunnel, type);
  convert(struct_v.distance_to_split, ros_v.distance_to_split, type);
  convert(struct_v.distance_to_merge, ros_v.distance_to_merge, type);
  convert(struct_v.upgrade_from_acc, ros_v.upgrade_from_acc, type);
  convert(struct_v.upgrade_from_scc, ros_v.upgrade_from_scc, type);
  convert(struct_v.road_condition_hint, ros_v.road_condition_hint, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiLineInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.l_line_marking_type, ros_v.l_line_marking_type, type);
  convert(struct_v.l_line_marking_dis, ros_v.l_line_marking_dis, type);
  convert(struct_v.l_line_marking_poly_coe_a0, ros_v.l_line_marking_poly_coe_a0, type);
  convert(struct_v.l_line_marking_poly_coe_a1, ros_v.l_line_marking_poly_coe_a1, type);
  convert(struct_v.l_line_marking_poly_coe_a2, ros_v.l_line_marking_poly_coe_a2, type);
  convert(struct_v.l_line_marking_start, ros_v.l_line_marking_start, type);
  convert(struct_v.l_line_marking_end, ros_v.l_line_marking_end, type);
  convert(struct_v.line_index, ros_v.line_index, type);
  convert(struct_v.line_id, ros_v.line_id, type);
  convert(struct_v.line_color, ros_v.line_color, type);
  convert(struct_v.line_width, ros_v.line_width, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiTargetOrientation &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.yaw, ros_v.yaw, type);
  convert(struct_v.pitch, ros_v.pitch, type);
  convert(struct_v.roll, ros_v.roll, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiObjInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.target_long_position, ros_v.target_long_position, type);
  convert(struct_v.target_lat_position, ros_v.target_lat_position, type);
  convert(struct_v.target_orientation, ros_v.target_orientation, type);
  convert(struct_v.target_track_id, ros_v.target_track_id, type);
  convert(struct_v.target_type, ros_v.target_type, type);
  convert(struct_v.motion_pattern, ros_v.motion_pattern, type);
  convert(struct_v.light_status, ros_v.light_status, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.velocity, ros_v.velocity, type);
  convert(struct_v.lane_id, ros_v.lane_id, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiApaSlotInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_slot_id, ros_v.apa_slot_id, type);
  convert(struct_v.apa_slot_type, ros_v.apa_slot_type, type);
  convert(struct_v.apa_slot_corner_points1, ros_v.apa_slot_corner_points1, type);
  convert(struct_v.apa_slot_corner_points2, ros_v.apa_slot_corner_points2, type);
  convert(struct_v.apa_slot_corner_points3, ros_v.apa_slot_corner_points3, type);
  convert(struct_v.apa_slot_corner_points4, ros_v.apa_slot_corner_points4, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiApaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_ble_current_cnt, ros_v.apa_ble_current_cnt, type);
  convert(struct_v.apa_ble_total_cnt, ros_v.apa_ble_total_cnt, type);
  convert(struct_v.apa_mode_status, ros_v.apa_mode_status, type);
  convert(struct_v.apa_status, ros_v.apa_status, type);
  convert(struct_v.apa_avm_status, ros_v.apa_avm_status, type);
  convert(struct_v.apa_close_door_req, ros_v.apa_close_door_req, type);
  convert(struct_v.apa_belted_req, ros_v.apa_belted_req, type);
  convert(struct_v.apa_brakeing_req, ros_v.apa_brakeing_req, type);
  convert(struct_v.apa_warning_req, ros_v.apa_warning_req, type);
  convert(struct_v.apa_park_demand, ros_v.apa_park_demand, type);
  convert(struct_v.apa_parkoutdir_indication, ros_v.apa_parkoutdir_indication, type);
  convert(struct_v.apa_move_object_indication, ros_v.apa_move_object_indication, type);
  convert(struct_v.apa_driver_operation_indication, ros_v.apa_driver_operation_indication, type);
  convert(struct_v.apa_parkout, ros_v.apa_parkout, type);
  convert(struct_v.apa_parktimeout, ros_v.apa_parktimeout, type);
  convert(struct_v.apa_pdc_audible_beeprate, ros_v.apa_pdc_audible_beeprate, type);
  convert(struct_v.apa_start, ros_v.apa_start, type);
  convert(struct_v.apa_menu, ros_v.apa_menu, type);
  convert(struct_v.apa_avaliable_state, ros_v.apa_avaliable_state, type);
  convert(struct_v.apa_parkout_condition, ros_v.apa_parkout_condition, type);
  convert(struct_v.apa_acm_lever_intervention, ros_v.apa_acm_lever_intervention, type);
  convert(struct_v.apa_icm_turnlight_fb, ros_v.apa_icm_turnlight_fb, type);
  convert(struct_v.apa_targear_req, ros_v.apa_targear_req, type);
  convert(struct_v.apa_parkspace_number, ros_v.apa_parkspace_number, type);
  for (size_t i0 = 0; i0 < ros_v.apa_slot_info.size(); i0++) {
	  convert(struct_v.apa_slot_info[i0], ros_v.apa_slot_info[i0], type);
  }
  convert(struct_v.apa_obj_number, ros_v.apa_obj_number, type);
  for (size_t i1 = 0; i1 < ros_v.apa_obj_info.size(); i1++) {
	  convert(struct_v.apa_obj_info[i1], ros_v.apa_obj_info[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiAccInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acc_notenable_reason, ros_v.acc_notenable_reason, type);
  convert(struct_v.acc_in_out_remind, ros_v.acc_in_out_remind, type);
  convert(struct_v.acc_status, ros_v.acc_status, type);
  convert(struct_v.acc_active_resp, ros_v.acc_active_resp, type);
  convert(struct_v.acc_driver_denied, ros_v.acc_driver_denied, type);
  convert(struct_v.acc_takeover_req_lv, ros_v.acc_takeover_req_lv, type);
  convert(struct_v.front_car_starts, ros_v.front_car_starts, type);
  convert(struct_v.acc_go_indicator, ros_v.acc_go_indicator, type);
  convert(struct_v.acc_driver_go, ros_v.acc_driver_go, type);
  convert(struct_v.acc_set_headway, ros_v.acc_set_headway, type);
  convert(struct_v.acc_set_speed, ros_v.acc_set_speed, type);
  convert(struct_v.intelligent_following, ros_v.intelligent_following, type);
  convert(struct_v.too_close_to_front, ros_v.too_close_to_front, type);
  convert(struct_v.acc_close_door_req, ros_v.acc_close_door_req, type);
  convert(struct_v.acc_belted_req, ros_v.acc_belted_req, type);
  convert(struct_v.acc_brakeing_req, ros_v.acc_brakeing_req, type);
  convert(struct_v.downgrade_from_scc, ros_v.downgrade_from_scc, type);
  convert(struct_v.downgrade_from_noa, ros_v.downgrade_from_noa, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiLaneChange &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lc_status, ros_v.lc_status, type);
  convert(struct_v.lc_direction, ros_v.lc_direction, type);
  convert(struct_v.lc_reason, ros_v.lc_reason, type);
  convert(struct_v.obstacle_id, ros_v.obstacle_id, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiIntelligentEvasion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.dodge_type, ros_v.dodge_type, type);
  convert(struct_v.object_id, ros_v.object_id, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiSccInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.scc_status, ros_v.scc_status, type);
  convert(struct_v.scc_active_resp, ros_v.scc_active_resp, type);
  convert(struct_v.scc_driver_denied, ros_v.scc_driver_denied, type);
  convert(struct_v.scc_hands_off_warning, ros_v.scc_hands_off_warning, type);
  convert(struct_v.scc_takeover_req_lv, ros_v.scc_takeover_req_lv, type);
  convert(struct_v.scc_line_detect_status, ros_v.scc_line_detect_status, type);
  convert(struct_v.scc_close_door_req, ros_v.scc_close_door_req, type);
  convert(struct_v.scc_belted_req, ros_v.scc_belted_req, type);
  convert(struct_v.scc_brakeing_req, ros_v.scc_brakeing_req, type);
  convert(struct_v.scc_notenable_reason, ros_v.scc_notenable_reason, type);
  convert(struct_v.scc_in_out_remind, ros_v.scc_in_out_remind, type);
  convert(struct_v.upgrade_from_acc, ros_v.upgrade_from_acc, type);
  convert(struct_v.downgrade_from_noa, ros_v.downgrade_from_noa, type);
  convert(struct_v.intelligent_evasion, ros_v.intelligent_evasion, type);
  convert(struct_v.lane_change, ros_v.lane_change, type);
  convert(struct_v.narrow_road_Tips, ros_v.narrow_road_Tips, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiTrafficLight &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.traffic_light_type, ros_v.traffic_light_type, type);
  convert(struct_v.traffic_light_color, ros_v.traffic_light_color, type);
  convert(struct_v.traffic_light_countdown_number, ros_v.traffic_light_countdown_number, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiOtherInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ldw_output_info, ros_v.ldw_output_info, type);
  convert(struct_v.ldp_output_info, ros_v.ldp_output_info, type);
  convert(struct_v.elk_output_info, ros_v.elk_output_info, type);
  convert(struct_v.tsr_output_info, ros_v.tsr_output_info, type);
  convert(struct_v.ihc_output_info, ros_v.ihc_output_info, type);
  convert(struct_v.alc_output_info, ros_v.alc_output_info, type);
  convert(struct_v.traffic_sign, ros_v.traffic_sign, type);
  convert(struct_v.traffic_light, ros_v.traffic_light, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiSensorInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.sensor_type, ros_v.sensor_type, type);
  convert(struct_v.sensor_state, ros_v.sensor_state, type);
}

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiSocOuter &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.hmi_common, ros_v.hmi_common, type);
  convert(struct_v.hmi_line_info_size, ros_v.hmi_line_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_line_info_size >= 0 && struct_v.hmi_line_info_size <= HMI_LINE_NUM) {
      ros_v.hmi_line_info.resize(struct_v.hmi_line_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_line_info_size=" << struct_v.hmi_line_info_size 
                << " not in range HMI_LINE_NUM=" << HMI_LINE_NUM 
                << std::endl;
      ros_v.hmi_line_info_size = HMI_LINE_NUM;
      ros_v.hmi_line_info.resize(HMI_LINE_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.hmi_line_info.size(); i0++) {
      convert(struct_v.hmi_line_info[i0], ros_v.hmi_line_info[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_line_info_size > HMI_LINE_NUM || ros_v.hmi_line_info_size < 0 || ros_v.hmi_line_info.size() > HMI_LINE_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_line_info_size=" << ros_v.hmi_line_info_size 
                << " ros_v.hmi_line_info.size()=" << ros_v.hmi_line_info.size()
                << " not in range HMI_LINE_NUM=" << HMI_LINE_NUM 
                << std::endl;
    }
    if (ros_v.hmi_line_info.size() > HMI_LINE_NUM) {
      for (size_t i0 = 0; i0 < HMI_LINE_NUM; i0++) {
        convert(struct_v.hmi_line_info[i0], ros_v.hmi_line_info[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.hmi_line_info.size(); i0++) {
        convert(struct_v.hmi_line_info[i0], ros_v.hmi_line_info[i0], type);
      }
    }
  }
  //
  convert(struct_v.hmi_obj_info_size, ros_v.hmi_obj_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_obj_info_size >= 0 && struct_v.hmi_obj_info_size <= HMI_OBJECT_NUM) {
      ros_v.hmi_obj_info.resize(struct_v.hmi_obj_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_obj_info_size=" << struct_v.hmi_obj_info_size 
                << " not in range HMI_OBJECT_NUM=" << HMI_OBJECT_NUM 
                << std::endl;
      ros_v.hmi_obj_info_size = HMI_OBJECT_NUM;
      ros_v.hmi_obj_info.resize(HMI_OBJECT_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.hmi_obj_info.size(); i1++) {
      convert(struct_v.hmi_obj_info[i1], ros_v.hmi_obj_info[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_obj_info_size > HMI_OBJECT_NUM || ros_v.hmi_obj_info_size < 0 || ros_v.hmi_obj_info.size() > HMI_OBJECT_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_obj_info_size=" << ros_v.hmi_obj_info_size 
                << " ros_v.hmi_obj_info.size()=" << ros_v.hmi_obj_info.size()
                << " not in range HMI_OBJECT_NUM=" << HMI_OBJECT_NUM 
                << std::endl;
    }
    if (ros_v.hmi_obj_info.size() > HMI_OBJECT_NUM) {
      for (size_t i1 = 0; i1 < HMI_OBJECT_NUM; i1++) {
        convert(struct_v.hmi_obj_info[i1], ros_v.hmi_obj_info[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.hmi_obj_info.size(); i1++) {
        convert(struct_v.hmi_obj_info[i1], ros_v.hmi_obj_info[i1], type);
      }
    }
  }
  //
  convert(struct_v.hmi_cipv_info, ros_v.hmi_cipv_info, type);
  convert(struct_v.hmi_apa_info, ros_v.hmi_apa_info, type);
  convert(struct_v.hmi_acc_info, ros_v.hmi_acc_info, type);
  convert(struct_v.hmi_scc_info, ros_v.hmi_scc_info, type);
  convert(struct_v.hmi_noa_info, ros_v.hmi_noa_info, type);
  convert(struct_v.hmi_other_info, ros_v.hmi_other_info, type);
  convert(struct_v.calib_info, ros_v.calib_info, type);
  convert(struct_v.sensor_info, ros_v.sensor_info, type);
}

