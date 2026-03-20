#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ADASFunctionDebugOutputInfo.h"
#include "struct_msgs_v2_10/ADASFunctionDebugOutputInfo.h"
#include "struct_msgs/AEBDebugOutputInfo.h"
#include "struct_msgs_v2_10/AEBDebugOutputInfo.h"
#include "struct_msgs/AEBPDebugOutputInfo.h"
#include "struct_msgs_v2_10/AEBPDebugOutputInfo.h"
#include "struct_msgs/AebObjInfo.h"
#include "struct_msgs_v2_10/AebObjInfo.h"
#include "struct_msgs/BSDDebugOutputInfo.h"
#include "struct_msgs_v2_10/BSDDebugOutputInfo.h"
#include "struct_msgs/CcrOutput.h"
#include "struct_msgs_v2_10/CcrOutput.h"
#include "struct_msgs/ControlAdaptorDebugOutputInfo.h"
#include "struct_msgs_v2_10/ControlAdaptorDebugOutputInfo.h"
#include "struct_msgs/DOWDebugOutputInfo.h"
#include "struct_msgs_v2_10/DOWDebugOutputInfo.h"
#include "struct_msgs/EthDebugOutputInfo.h"
#include "struct_msgs_v2_10/EthDebugOutputInfo.h"
#include "struct_msgs/FCTADebugOutputInfo.h"
#include "struct_msgs_v2_10/FCTADebugOutputInfo.h"
#include "struct_msgs/FCTBDebugOutputInfo.h"
#include "struct_msgs_v2_10/FCTBDebugOutputInfo.h"
#include "struct_msgs/FCWDebugOutputInfo.h"
#include "struct_msgs_v2_10/FCWDebugOutputInfo.h"
#include "struct_msgs/LCADebugOutputInfo.h"
#include "struct_msgs_v2_10/LCADebugOutputInfo.h"
#include "struct_msgs/PCWDebugOutputInfo.h"
#include "struct_msgs_v2_10/PCWDebugOutputInfo.h"
#include "struct_msgs/RCTADebugOutputInfo.h"
#include "struct_msgs_v2_10/RCTADebugOutputInfo.h"
#include "struct_msgs/RCTBDebugOutputInfo.h"
#include "struct_msgs_v2_10/RCTBDebugOutputInfo.h"
#include "struct_msgs/RCWDebugOutputInfo.h"
#include "struct_msgs_v2_10/RCWDebugOutputInfo.h"
#include "struct_msgs/VruOutput.h"
#include "struct_msgs_v2_10/VruOutput.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ADASFunctionDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.fcw_output_info, ros_v.fcw_output_info, type);
	convert(old_ros_v.pcw_output_info, ros_v.pcw_output_info, type);
	convert(old_ros_v.aeb_output_info, ros_v.aeb_output_info, type);
	convert(old_ros_v.aebp_output_info, ros_v.aebp_output_info, type);
	convert(old_ros_v.bsd_output_info, ros_v.bsd_output_info, type);
	convert(old_ros_v.lca_output_info, ros_v.lca_output_info, type);
	convert(old_ros_v.dow_output_info, ros_v.dow_output_info, type);
	convert(old_ros_v.rcw_output_info, ros_v.rcw_output_info, type);
	convert(old_ros_v.fcta_output_info, ros_v.fcta_output_info, type);
	convert(old_ros_v.rcta_output_info, ros_v.rcta_output_info, type);
	convert(old_ros_v.fctb_output_info, ros_v.fctb_output_info, type);
	convert(old_ros_v.rctb_output_info, ros_v.rctb_output_info, type);
	convert(old_ros_v.control_adaptor_debug_output_info, ros_v.control_adaptor_debug_output_info, type);
	convert(old_ros_v.ccr_output_info, ros_v.ccr_output_info, type);
	convert(old_ros_v.vru_output_info, ros_v.vru_output_info, type);
	convert(old_ros_v.eth_debug_output_info, ros_v.eth_debug_output_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AEBDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.aeb_state, ros_v.aeb_state, type);
	convert(old_ros_v.aeb_deceleration_request_status, ros_v.aeb_deceleration_request_status, type);
	convert(old_ros_v.aeb_deceleration_request_value, ros_v.aeb_deceleration_request_value, type);
	convert(old_ros_v.aeb_hold_request_status, ros_v.aeb_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AEBPDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.aebp_state, ros_v.aebp_state, type);
	convert(old_ros_v.aebp_deceleration_request_status, ros_v.aebp_deceleration_request_status, type);
	convert(old_ros_v.aebp_deceleration_request_value, ros_v.aebp_deceleration_request_value, type);
	convert(old_ros_v.aebp_hold_request_status, ros_v.aebp_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AebObjInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.obj_vaild, ros_v.obj_vaild, type);
	convert(old_ros_v.index, ros_v.index, type);
	convert(old_ros_v.obj_id, ros_v.obj_id, type);
	convert(old_ros_v.obj_x, ros_v.obj_x, type);
	convert(old_ros_v.obj_y, ros_v.obj_y, type);
	convert(old_ros_v.obj_v_x, ros_v.obj_v_x, type);
	convert(old_ros_v.obj_v_y, ros_v.obj_v_y, type);
	convert(old_ros_v.obj_a_x, ros_v.obj_a_x, type);
	convert(old_ros_v.obj_a_y, ros_v.obj_a_y, type);
	convert(old_ros_v.obj_length, ros_v.obj_length, type);
	convert(old_ros_v.obj_width, ros_v.obj_width, type);
	convert(old_ros_v.obj_heading_angle, ros_v.obj_heading_angle, type);
	convert(old_ros_v.obj_class, ros_v.obj_class, type);
	convert(old_ros_v.obj_ettc, ros_v.obj_ettc, type);
	convert(old_ros_v.obj_turn_out_a_y, ros_v.obj_turn_out_a_y, type);
	convert(old_ros_v.fcw_level1_threshold, ros_v.fcw_level1_threshold, type);
	convert(old_ros_v.fcw_level2_threshold, ros_v.fcw_level2_threshold, type);
	convert(old_ros_v.aeb_soft_threshold, ros_v.aeb_soft_threshold, type);
	convert(old_ros_v.aeb_hard_threshold, ros_v.aeb_hard_threshold, type);
	convert(old_ros_v.fcw_level1_alert, ros_v.fcw_level1_alert, type);
	convert(old_ros_v.fcw_level2_alert, ros_v.fcw_level2_alert, type);
	convert(old_ros_v.aeb_soft_alert, ros_v.aeb_soft_alert, type);
	convert(old_ros_v.aeb_hard_alert, ros_v.aeb_hard_alert, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::BSDDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.bsd_state, ros_v.bsd_state, type);
	convert(old_ros_v.bsd_left_warning_level, ros_v.bsd_left_warning_level, type);
	convert(old_ros_v.bsd_right_warning_level, ros_v.bsd_right_warning_level, type);
	convert(old_ros_v.bsd_left_fence_judge, ros_v.bsd_left_fence_judge, type);
	convert(old_ros_v.bsd_right_fence_judge, ros_v.bsd_right_fence_judge, type);
	convert(old_ros_v.bsd_left_fence_reprocess_judge, ros_v.bsd_left_fence_reprocess_judge, type);
	convert(old_ros_v.bsd_right_fence_reprocess_judge, ros_v.bsd_right_fence_reprocess_judge, type);
	convert(old_ros_v.bsd_left_warning_front, ros_v.bsd_left_warning_front, type);
	convert(old_ros_v.bsd_right_warning_front, ros_v.bsd_right_warning_front, type);
	convert(old_ros_v.bsd_fr_warning_code_total, ros_v.bsd_fr_warning_code_total, type);
	convert(old_ros_v.bsd_rr_warning_code_total, ros_v.bsd_rr_warning_code_total, type);
	convert(old_ros_v.bsd_fl_warning_code_total, ros_v.bsd_fl_warning_code_total, type);
	convert(old_ros_v.bsd_rl_warning_code_total, ros_v.bsd_rl_warning_code_total, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CcrOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.cipv_1nd, ros_v.cipv_1nd, type);
	convert(old_ros_v.cipv_2nd, ros_v.cipv_2nd, type);
	convert(old_ros_v.left_1nd, ros_v.left_1nd, type);
	convert(old_ros_v.right_1nd, ros_v.right_1nd, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ControlAdaptorDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lat_ctrl_desired_angle_req_status, ros_v.lat_ctrl_desired_angle_req_status, type);
	convert(old_ros_v.lat_ctrl_desired_angle, ros_v.lat_ctrl_desired_angle, type);
	convert(old_ros_v.lat_ctrl_desired_angle_dt, ros_v.lat_ctrl_desired_angle_dt, type);
	convert(old_ros_v.lat_ctrl_desired_angle_error, ros_v.lat_ctrl_desired_angle_error, type);
	convert(old_ros_v.lat_ctrl_desired_angle_error_dt, ros_v.lat_ctrl_desired_angle_error_dt, type);
	convert(old_ros_v.lat_ctrl_trq_feedforward, ros_v.lat_ctrl_trq_feedforward, type);
	convert(old_ros_v.lat_ctrl_trq_feedback, ros_v.lat_ctrl_trq_feedback, type);
	convert(old_ros_v.lat_ctrl_trq_feedback_p, ros_v.lat_ctrl_trq_feedback_p, type);
	convert(old_ros_v.lat_ctrl_trq_feedback_p_gain, ros_v.lat_ctrl_trq_feedback_p_gain, type);
	convert(old_ros_v.lat_ctrl_trq_feedback_i, ros_v.lat_ctrl_trq_feedback_i, type);
	convert(old_ros_v.lat_ctrl_trq_feedback_i_gain, ros_v.lat_ctrl_trq_feedback_i_gain, type);
	convert(old_ros_v.lat_ctrl_trq_feedback_d, ros_v.lat_ctrl_trq_feedback_d, type);
	convert(old_ros_v.lat_ctrl_trq_req_status, ros_v.lat_ctrl_trq_req_status, type);
	convert(old_ros_v.lat_ctrl_trq_req_value, ros_v.lat_ctrl_trq_req_value, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::DOWDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.dow_state, ros_v.dow_state, type);
	convert(old_ros_v.dow_front_left_warning_level, ros_v.dow_front_left_warning_level, type);
	convert(old_ros_v.dow_front_right_warning_level, ros_v.dow_front_right_warning_level, type);
	convert(old_ros_v.dow_rear_left_warning_level, ros_v.dow_rear_left_warning_level, type);
	convert(old_ros_v.dow_rear_right_warning_level, ros_v.dow_rear_right_warning_level, type);
	convert(old_ros_v.dow_rr_warning_code_total, ros_v.dow_rr_warning_code_total, type);
	convert(old_ros_v.dow_rl_warning_code_total, ros_v.dow_rl_warning_code_total, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EthDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ctrl_timestamp, ros_v.ctrl_timestamp, type);
	convert(old_ros_v.fusion_road_timestamp, ros_v.fusion_road_timestamp, type);
	convert(old_ros_v.fusion_objects_timestamp, ros_v.fusion_objects_timestamp, type);
	convert(old_ros_v.hmi_timestamp, ros_v.hmi_timestamp, type);
	convert(old_ros_v.corner_radar_fl_timestamp, ros_v.corner_radar_fl_timestamp, type);
	convert(old_ros_v.corner_radar_fr_timestamp, ros_v.corner_radar_fr_timestamp, type);
	convert(old_ros_v.corner_radar_rl_timestamp, ros_v.corner_radar_rl_timestamp, type);
	convert(old_ros_v.corner_radar_rr_timestamp, ros_v.corner_radar_rr_timestamp, type);
	convert(old_ros_v.pilot_acc_to_veh, ros_v.pilot_acc_to_veh, type);
	convert(old_ros_v.pilot_torque_to_veh, ros_v.pilot_torque_to_veh, type);
	convert(old_ros_v.pilot_steer_angle_to_veh, ros_v.pilot_steer_angle_to_veh, type);
	convert(old_ros_v.park_acc_to_veh, ros_v.park_acc_to_veh, type);
	convert(old_ros_v.park_steer_to_veh, ros_v.park_steer_to_veh, type);
	convert(old_ros_v.park_torque_to_veh, ros_v.park_torque_to_veh, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCTADebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fcta_state, ros_v.fcta_state, type);
	convert(old_ros_v.fcta_left_warning_level, ros_v.fcta_left_warning_level, type);
	convert(old_ros_v.fcta_right_warning_level, ros_v.fcta_right_warning_level, type);
	convert(old_ros_v.fcta_enable_code, ros_v.fcta_enable_code, type);
	convert(old_ros_v.fcta_disable_code, ros_v.fcta_disable_code, type);
	convert(old_ros_v.fcta_objs_state_l1, ros_v.fcta_objs_state_l1, type);
	convert(old_ros_v.fcta_objs_state_r1, ros_v.fcta_objs_state_r1, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCTBDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fctb_state, ros_v.fctb_state, type);
	convert(old_ros_v.fctb_deceleration_request_status, ros_v.fctb_deceleration_request_status, type);
	convert(old_ros_v.fctb_deceleration_request_value, ros_v.fctb_deceleration_request_value, type);
	convert(old_ros_v.fctb_hold_request_status, ros_v.fctb_hold_request_status, type);
	convert(old_ros_v.fctb_enable_code, ros_v.fctb_enable_code, type);
	convert(old_ros_v.fctb_disable_code, ros_v.fctb_disable_code, type);
	convert(old_ros_v.fctb_brake_code, ros_v.fctb_brake_code, type);
	convert(old_ros_v.fctb_autohold_code, ros_v.fctb_autohold_code, type);
	convert(old_ros_v.fctb_brake_kick_code, ros_v.fctb_brake_kick_code, type);
	convert(old_ros_v.fctb_autohold_kick_code, ros_v.fctb_autohold_kick_code, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCWDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fcw_state, ros_v.fcw_state, type);
	convert(old_ros_v.fcw_warning_level, ros_v.fcw_warning_level, type);
	convert(old_ros_v.fcw_Prefill_request_status, ros_v.fcw_Prefill_request_status, type);
	convert(old_ros_v.fcw_jerk_request_status, ros_v.fcw_jerk_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LCADebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lca_state, ros_v.lca_state, type);
	convert(old_ros_v.lca_left_warning_level, ros_v.lca_left_warning_level, type);
	convert(old_ros_v.lca_right_warning_level, ros_v.lca_right_warning_level, type);
	convert(old_ros_v.lca_left_area_car_flag, ros_v.lca_left_area_car_flag, type);
	convert(old_ros_v.lca_right_area_car_flag, ros_v.lca_right_area_car_flag, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PCWDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.pcw_state, ros_v.pcw_state, type);
	convert(old_ros_v.pcw_warning_level, ros_v.pcw_warning_level, type);
	convert(old_ros_v.pcw_prefill_request_status, ros_v.pcw_prefill_request_status, type);
	convert(old_ros_v.pcw_jerk_request_status, ros_v.pcw_jerk_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCTADebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rcta_state, ros_v.rcta_state, type);
	convert(old_ros_v.rcta_left_warning_level, ros_v.rcta_left_warning_level, type);
	convert(old_ros_v.rcta_right_warning_level, ros_v.rcta_right_warning_level, type);
	convert(old_ros_v.rcta_enable_code, ros_v.rcta_enable_code, type);
	convert(old_ros_v.rcta_disable_code, ros_v.rcta_disable_code, type);
	convert(old_ros_v.rcta_objs_state_l1, ros_v.rcta_objs_state_l1, type);
	convert(old_ros_v.rcta_objs_state_r1, ros_v.rcta_objs_state_r1, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCTBDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rctb_state, ros_v.rctb_state, type);
	convert(old_ros_v.rctb_deceleration_request_status, ros_v.rctb_deceleration_request_status, type);
	convert(old_ros_v.rctb_deceleration_request_value, ros_v.rctb_deceleration_request_value, type);
	convert(old_ros_v.rctb_hold_request_status, ros_v.rctb_hold_request_status, type);
	convert(old_ros_v.rctb_enable_code, ros_v.rctb_enable_code, type);
	convert(old_ros_v.rctb_disable_code, ros_v.rctb_disable_code, type);
	convert(old_ros_v.rctb_brake_code, ros_v.rctb_brake_code, type);
	convert(old_ros_v.rctb_autohold_code, ros_v.rctb_autohold_code, type);
	convert(old_ros_v.rctb_brake_kick_code, ros_v.rctb_brake_kick_code, type);
	convert(old_ros_v.rctb_autohold_kick_code, ros_v.rctb_autohold_kick_code, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCWDebugOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rcw_state, ros_v.rcw_state, type);
	convert(old_ros_v.rcw_warning_level, ros_v.rcw_warning_level, type);
	convert(old_ros_v.rcw_enable_code, ros_v.rcw_enable_code, type);
	convert(old_ros_v.rcw_disable_code, ros_v.rcw_disable_code, type);
	convert(old_ros_v.rcw_left_warning_flag, ros_v.rcw_left_warning_flag, type);
	convert(old_ros_v.rcw_right_warning_flag, ros_v.rcw_right_warning_flag, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::VruOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.obj_vaild, ros_v.obj_vaild, type);
	convert(old_ros_v.index, ros_v.index, type);
	convert(old_ros_v.obj_id, ros_v.obj_id, type);
	convert(old_ros_v.obj_x, ros_v.obj_x, type);
	convert(old_ros_v.obj_y, ros_v.obj_y, type);
	convert(old_ros_v.obj_v_x, ros_v.obj_v_x, type);
	convert(old_ros_v.obj_v_y, ros_v.obj_v_y, type);
	convert(old_ros_v.obj_a_x, ros_v.obj_a_x, type);
	convert(old_ros_v.obj_a_y, ros_v.obj_a_y, type);
	convert(old_ros_v.obj_length, ros_v.obj_length, type);
	convert(old_ros_v.obj_width, ros_v.obj_width, type);
	convert(old_ros_v.obj_heading_angle, ros_v.obj_heading_angle, type);
	convert(old_ros_v.obj_class, ros_v.obj_class, type);
	convert(old_ros_v.obj_ettc, ros_v.obj_ettc, type);
	convert(old_ros_v.pcw_level1_threshold, ros_v.pcw_level1_threshold, type);
	convert(old_ros_v.pcw_level2_threshold, ros_v.pcw_level2_threshold, type);
	convert(old_ros_v.aebp_soft_threshold, ros_v.aebp_soft_threshold, type);
	convert(old_ros_v.aebp_hard_threshold, ros_v.aebp_hard_threshold, type);
	convert(old_ros_v.pcw_level1_alert, ros_v.pcw_level1_alert, type);
	convert(old_ros_v.pcw_level2_alert, ros_v.pcw_level2_alert, type);
	convert(old_ros_v.aebp_soft_alert, ros_v.aebp_soft_alert, type);
	convert(old_ros_v.aebp_hard_alert, ros_v.aebp_hard_alert, type);
}

REG_CONVERT_SINGLE(_iflytek_adas_function_debug_info_converter, "/iflytek/adas_function/debug_info", ADASFunctionDebugOutputInfo);
