#pragma once

#include "base_convert.h"
#include "c/adas_function_debug_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FCWDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fcw_state, ros_v.fcw_state, type);
  convert(struct_v.fcw_warning_level, ros_v.fcw_warning_level, type);
  convert(struct_v.fcw_Prefill_request_status, ros_v.fcw_Prefill_request_status, type);
  convert(struct_v.fcw_jerk_request_status, ros_v.fcw_jerk_request_status, type);
}

template <typename T2>
void convert(iflyauto::PCWDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pcw_state, ros_v.pcw_state, type);
  convert(struct_v.pcw_warning_level, ros_v.pcw_warning_level, type);
  convert(struct_v.pcw_prefill_request_status, ros_v.pcw_prefill_request_status, type);
  convert(struct_v.pcw_jerk_request_status, ros_v.pcw_jerk_request_status, type);
}

template <typename T2>
void convert(iflyauto::AEBDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.aeb_state, ros_v.aeb_state, type);
  convert(struct_v.aeb_deceleration_request_status, ros_v.aeb_deceleration_request_status, type);
  convert(struct_v.aeb_deceleration_request_value, ros_v.aeb_deceleration_request_value, type);
  convert(struct_v.aeb_hold_request_status, ros_v.aeb_hold_request_status, type);
}

template <typename T2>
void convert(iflyauto::AEBPDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.aebp_state, ros_v.aebp_state, type);
  convert(struct_v.aebp_deceleration_request_status, ros_v.aebp_deceleration_request_status, type);
  convert(struct_v.aebp_deceleration_request_value, ros_v.aebp_deceleration_request_value, type);
  convert(struct_v.aebp_hold_request_status, ros_v.aebp_hold_request_status, type);
}

template <typename T2>
void convert(iflyauto::BSDDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.bsd_state, ros_v.bsd_state, type);
  convert(struct_v.bsd_left_warning_level, ros_v.bsd_left_warning_level, type);
  convert(struct_v.bsd_right_warning_level, ros_v.bsd_right_warning_level, type);
  convert(struct_v.bsd_left_fence_judge, ros_v.bsd_left_fence_judge, type);
  convert(struct_v.bsd_right_fence_judge, ros_v.bsd_right_fence_judge, type);
  convert(struct_v.bsd_left_fence_reprocess_judge, ros_v.bsd_left_fence_reprocess_judge, type);
  convert(struct_v.bsd_right_fence_reprocess_judge, ros_v.bsd_right_fence_reprocess_judge, type);
  convert(struct_v.bsd_left_warning_front, ros_v.bsd_left_warning_front, type);
  convert(struct_v.bsd_right_warning_front, ros_v.bsd_right_warning_front, type);
  convert(struct_v.bsd_fr_warning_code_total, ros_v.bsd_fr_warning_code_total, type);
  convert(struct_v.bsd_rr_warning_code_total, ros_v.bsd_rr_warning_code_total, type);
  convert(struct_v.bsd_fl_warning_code_total, ros_v.bsd_fl_warning_code_total, type);
  convert(struct_v.bsd_rl_warning_code_total, ros_v.bsd_rl_warning_code_total, type);
}

template <typename T2>
void convert(iflyauto::LCADebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lca_state, ros_v.lca_state, type);
  convert(struct_v.lca_left_warning_level, ros_v.lca_left_warning_level, type);
  convert(struct_v.lca_right_warning_level, ros_v.lca_right_warning_level, type);
  convert(struct_v.lca_left_area_car_flag, ros_v.lca_left_area_car_flag, type);
  convert(struct_v.lca_right_area_car_flag, ros_v.lca_right_area_car_flag, type);
}

template <typename T2>
void convert(iflyauto::DOWDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.dow_state, ros_v.dow_state, type);
  convert(struct_v.dow_front_left_warning_level, ros_v.dow_front_left_warning_level, type);
  convert(struct_v.dow_front_right_warning_level, ros_v.dow_front_right_warning_level, type);
  convert(struct_v.dow_rear_left_warning_level, ros_v.dow_rear_left_warning_level, type);
  convert(struct_v.dow_rear_right_warning_level, ros_v.dow_rear_right_warning_level, type);
  convert(struct_v.dow_rr_warning_code_total, ros_v.dow_rr_warning_code_total, type);
  convert(struct_v.dow_rl_warning_code_total, ros_v.dow_rl_warning_code_total, type);
}

template <typename T2>
void convert(iflyauto::RCWDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rcw_state, ros_v.rcw_state, type);
  convert(struct_v.rcw_warning_level, ros_v.rcw_warning_level, type);
  convert(struct_v.rcw_enable_code, ros_v.rcw_enable_code, type);
  convert(struct_v.rcw_disable_code, ros_v.rcw_disable_code, type);
  convert(struct_v.rcw_left_warning_flag, ros_v.rcw_left_warning_flag, type);
  convert(struct_v.rcw_right_warning_flag, ros_v.rcw_right_warning_flag, type);
}

template <typename T2>
void convert(iflyauto::FCTADebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fcta_state, ros_v.fcta_state, type);
  convert(struct_v.fcta_left_warning_level, ros_v.fcta_left_warning_level, type);
  convert(struct_v.fcta_right_warning_level, ros_v.fcta_right_warning_level, type);
  convert(struct_v.fcta_enable_code, ros_v.fcta_enable_code, type);
  convert(struct_v.fcta_disable_code, ros_v.fcta_disable_code, type);
  convert(struct_v.fcta_objs_state_l1, ros_v.fcta_objs_state_l1, type);
  convert(struct_v.fcta_objs_state_r1, ros_v.fcta_objs_state_r1, type);
}

template <typename T2>
void convert(iflyauto::RCTADebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rcta_state, ros_v.rcta_state, type);
  convert(struct_v.rcta_left_warning_level, ros_v.rcta_left_warning_level, type);
  convert(struct_v.rcta_right_warning_level, ros_v.rcta_right_warning_level, type);
  convert(struct_v.rcta_enable_code, ros_v.rcta_enable_code, type);
  convert(struct_v.rcta_disable_code, ros_v.rcta_disable_code, type);
  convert(struct_v.rcta_objs_state_l1, ros_v.rcta_objs_state_l1, type);
  convert(struct_v.rcta_objs_state_r1, ros_v.rcta_objs_state_r1, type);
}

template <typename T2>
void convert(iflyauto::FCTBDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fctb_state, ros_v.fctb_state, type);
  convert(struct_v.fctb_deceleration_request_status, ros_v.fctb_deceleration_request_status, type);
  convert(struct_v.fctb_deceleration_request_value, ros_v.fctb_deceleration_request_value, type);
  convert(struct_v.fctb_hold_request_status, ros_v.fctb_hold_request_status, type);
  convert(struct_v.fctb_enable_code, ros_v.fctb_enable_code, type);
  convert(struct_v.fctb_disable_code, ros_v.fctb_disable_code, type);
  convert(struct_v.fctb_brake_code, ros_v.fctb_brake_code, type);
  convert(struct_v.fctb_autohold_code, ros_v.fctb_autohold_code, type);
  convert(struct_v.fctb_brake_kick_code, ros_v.fctb_brake_kick_code, type);
  convert(struct_v.fctb_autohold_kick_code, ros_v.fctb_autohold_kick_code, type);
}

template <typename T2>
void convert(iflyauto::RCTBDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rctb_state, ros_v.rctb_state, type);
  convert(struct_v.rctb_deceleration_request_status, ros_v.rctb_deceleration_request_status, type);
  convert(struct_v.rctb_deceleration_request_value, ros_v.rctb_deceleration_request_value, type);
  convert(struct_v.rctb_hold_request_status, ros_v.rctb_hold_request_status, type);
  convert(struct_v.rctb_enable_code, ros_v.rctb_enable_code, type);
  convert(struct_v.rctb_disable_code, ros_v.rctb_disable_code, type);
  convert(struct_v.rctb_brake_code, ros_v.rctb_brake_code, type);
  convert(struct_v.rctb_autohold_code, ros_v.rctb_autohold_code, type);
  convert(struct_v.rctb_brake_kick_code, ros_v.rctb_brake_kick_code, type);
  convert(struct_v.rctb_autohold_kick_code, ros_v.rctb_autohold_kick_code, type);
}

template <typename T2>
void convert(iflyauto::ControlAdaptorDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lat_ctrl_desired_angle_req_status, ros_v.lat_ctrl_desired_angle_req_status, type);
  convert(struct_v.lat_ctrl_desired_angle, ros_v.lat_ctrl_desired_angle, type);
  convert(struct_v.lat_ctrl_desired_angle_dt, ros_v.lat_ctrl_desired_angle_dt, type);
  convert(struct_v.lat_ctrl_desired_angle_error, ros_v.lat_ctrl_desired_angle_error, type);
  convert(struct_v.lat_ctrl_desired_angle_error_dt, ros_v.lat_ctrl_desired_angle_error_dt, type);
  convert(struct_v.lat_ctrl_trq_feedforward, ros_v.lat_ctrl_trq_feedforward, type);
  convert(struct_v.lat_ctrl_trq_feedback, ros_v.lat_ctrl_trq_feedback, type);
  convert(struct_v.lat_ctrl_trq_feedback_p, ros_v.lat_ctrl_trq_feedback_p, type);
  convert(struct_v.lat_ctrl_trq_feedback_p_gain, ros_v.lat_ctrl_trq_feedback_p_gain, type);
  convert(struct_v.lat_ctrl_trq_feedback_i, ros_v.lat_ctrl_trq_feedback_i, type);
  convert(struct_v.lat_ctrl_trq_feedback_i_gain, ros_v.lat_ctrl_trq_feedback_i_gain, type);
  convert(struct_v.lat_ctrl_trq_feedback_d, ros_v.lat_ctrl_trq_feedback_d, type);
  convert(struct_v.lat_ctrl_trq_req_status, ros_v.lat_ctrl_trq_req_status, type);
  convert(struct_v.lat_ctrl_trq_req_value, ros_v.lat_ctrl_trq_req_value, type);
}

template <typename T2>
void convert(iflyauto::AebObjInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_vaild, ros_v.obj_vaild, type);
  convert(struct_v.index, ros_v.index, type);
  convert(struct_v.obj_id, ros_v.obj_id, type);
  convert(struct_v.obj_x, ros_v.obj_x, type);
  convert(struct_v.obj_y, ros_v.obj_y, type);
  convert(struct_v.obj_v_x, ros_v.obj_v_x, type);
  convert(struct_v.obj_v_y, ros_v.obj_v_y, type);
  convert(struct_v.obj_a_x, ros_v.obj_a_x, type);
  convert(struct_v.obj_a_y, ros_v.obj_a_y, type);
  convert(struct_v.obj_length, ros_v.obj_length, type);
  convert(struct_v.obj_width, ros_v.obj_width, type);
  convert(struct_v.obj_heading_angle, ros_v.obj_heading_angle, type);
  convert(struct_v.obj_class, ros_v.obj_class, type);
  convert(struct_v.obj_ettc, ros_v.obj_ettc, type);
  convert(struct_v.obj_turn_out_a_y, ros_v.obj_turn_out_a_y, type);
  convert(struct_v.fcw_level1_threshold, ros_v.fcw_level1_threshold, type);
  convert(struct_v.fcw_level2_threshold, ros_v.fcw_level2_threshold, type);
  convert(struct_v.aeb_soft_threshold, ros_v.aeb_soft_threshold, type);
  convert(struct_v.aeb_hard_threshold, ros_v.aeb_hard_threshold, type);
  convert(struct_v.fcw_level1_alert, ros_v.fcw_level1_alert, type);
  convert(struct_v.fcw_level2_alert, ros_v.fcw_level2_alert, type);
  convert(struct_v.aeb_soft_alert, ros_v.aeb_soft_alert, type);
  convert(struct_v.aeb_hard_alert, ros_v.aeb_hard_alert, type);
}

template <typename T2>
void convert(iflyauto::CcrOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.cipv_1nd, ros_v.cipv_1nd, type);
  convert(struct_v.cipv_2nd, ros_v.cipv_2nd, type);
  convert(struct_v.left_1nd, ros_v.left_1nd, type);
  convert(struct_v.right_1nd, ros_v.right_1nd, type);
}

template <typename T2>
void convert(iflyauto::VruOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_vaild, ros_v.obj_vaild, type);
  convert(struct_v.index, ros_v.index, type);
  convert(struct_v.obj_id, ros_v.obj_id, type);
  convert(struct_v.obj_x, ros_v.obj_x, type);
  convert(struct_v.obj_y, ros_v.obj_y, type);
  convert(struct_v.obj_v_x, ros_v.obj_v_x, type);
  convert(struct_v.obj_v_y, ros_v.obj_v_y, type);
  convert(struct_v.obj_a_x, ros_v.obj_a_x, type);
  convert(struct_v.obj_a_y, ros_v.obj_a_y, type);
  convert(struct_v.obj_length, ros_v.obj_length, type);
  convert(struct_v.obj_width, ros_v.obj_width, type);
  convert(struct_v.obj_heading_angle, ros_v.obj_heading_angle, type);
  convert(struct_v.obj_class, ros_v.obj_class, type);
  convert(struct_v.obj_ettc, ros_v.obj_ettc, type);
  convert(struct_v.pcw_level1_threshold, ros_v.pcw_level1_threshold, type);
  convert(struct_v.pcw_level2_threshold, ros_v.pcw_level2_threshold, type);
  convert(struct_v.aebp_soft_threshold, ros_v.aebp_soft_threshold, type);
  convert(struct_v.aebp_hard_threshold, ros_v.aebp_hard_threshold, type);
  convert(struct_v.pcw_level1_alert, ros_v.pcw_level1_alert, type);
  convert(struct_v.pcw_level2_alert, ros_v.pcw_level2_alert, type);
  convert(struct_v.aebp_soft_alert, ros_v.aebp_soft_alert, type);
  convert(struct_v.aebp_hard_alert, ros_v.aebp_hard_alert, type);
}

template <typename T2>
void convert(iflyauto::EthDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ctrl_timestamp, ros_v.ctrl_timestamp, type);
  convert(struct_v.fusion_road_timestamp, ros_v.fusion_road_timestamp, type);
  convert(struct_v.fusion_objects_timestamp, ros_v.fusion_objects_timestamp, type);
  convert(struct_v.hmi_timestamp, ros_v.hmi_timestamp, type);
  convert(struct_v.corner_radar_fl_timestamp, ros_v.corner_radar_fl_timestamp, type);
  convert(struct_v.corner_radar_fr_timestamp, ros_v.corner_radar_fr_timestamp, type);
  convert(struct_v.corner_radar_rl_timestamp, ros_v.corner_radar_rl_timestamp, type);
  convert(struct_v.corner_radar_rr_timestamp, ros_v.corner_radar_rr_timestamp, type);
  convert(struct_v.pilot_acc_to_veh, ros_v.pilot_acc_to_veh, type);
  convert(struct_v.pilot_torque_to_veh, ros_v.pilot_torque_to_veh, type);
  convert(struct_v.pilot_steer_angle_to_veh, ros_v.pilot_steer_angle_to_veh, type);
  convert(struct_v.park_acc_to_veh, ros_v.park_acc_to_veh, type);
  convert(struct_v.park_steer_to_veh, ros_v.park_steer_to_veh, type);
  convert(struct_v.park_torque_to_veh, ros_v.park_torque_to_veh, type);
}

template <typename T2>
void convert(iflyauto::ADASFunctionDebugOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.fcw_output_info, ros_v.fcw_output_info, type);
  convert(struct_v.pcw_output_info, ros_v.pcw_output_info, type);
  convert(struct_v.aeb_output_info, ros_v.aeb_output_info, type);
  convert(struct_v.aebp_output_info, ros_v.aebp_output_info, type);
  convert(struct_v.bsd_output_info, ros_v.bsd_output_info, type);
  convert(struct_v.lca_output_info, ros_v.lca_output_info, type);
  convert(struct_v.dow_output_info, ros_v.dow_output_info, type);
  convert(struct_v.rcw_output_info, ros_v.rcw_output_info, type);
  convert(struct_v.fcta_output_info, ros_v.fcta_output_info, type);
  convert(struct_v.rcta_output_info, ros_v.rcta_output_info, type);
  convert(struct_v.fctb_output_info, ros_v.fctb_output_info, type);
  convert(struct_v.rctb_output_info, ros_v.rctb_output_info, type);
  convert(struct_v.control_adaptor_debug_output_info, ros_v.control_adaptor_debug_output_info, type);
  convert(struct_v.ccr_output_info, ros_v.ccr_output_info, type);
  convert(struct_v.vru_output_info, ros_v.vru_output_info, type);
  convert(struct_v.eth_debug_output_info, ros_v.eth_debug_output_info, type);
}

