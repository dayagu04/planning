#pragma once

#include "base_convert.h"
#include "c/adas_function_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::RiskObjInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_valid, ros_v.obj_valid, type);
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.light_type, ros_v.light_type, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_acceleration, ros_v.relative_acceleration, type);
  convert(struct_v.relative_center_position, ros_v.relative_center_position, type);
  convert(struct_v.relative_heading_angle, ros_v.relative_heading_angle, type);
  convert(struct_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
}

template <typename T2>
void convert(iflyauto::FCWOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fcw_state, ros_v.fcw_state, type);
  convert(struct_v.fcw_warning_level, ros_v.fcw_warning_level, type);
  convert(struct_v.fcw_Prefill_request_status, ros_v.fcw_Prefill_request_status, type);
  convert(struct_v.fcw_jerk_request_status, ros_v.fcw_jerk_request_status, type);
  convert(struct_v.fcw_risk_obj, ros_v.fcw_risk_obj, type);
  convert(struct_v.fcw_headway_warning_request, ros_v.fcw_headway_warning_request, type);
}

template <typename T2>
void convert(iflyauto::PCWOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pcw_state, ros_v.pcw_state, type);
  convert(struct_v.pcw_warning_level, ros_v.pcw_warning_level, type);
  convert(struct_v.pcw_prefill_request_status, ros_v.pcw_prefill_request_status, type);
  convert(struct_v.pcw_jerk_request_status, ros_v.pcw_jerk_request_status, type);
  convert(struct_v.pcw_risk_obj, ros_v.pcw_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::AEBOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.aeb_state, ros_v.aeb_state, type);
  convert(struct_v.aeb_deceleration_request_status, ros_v.aeb_deceleration_request_status, type);
  convert(struct_v.aeb_deceleration_request_value, ros_v.aeb_deceleration_request_value, type);
  convert(struct_v.aeb_hold_request_status, ros_v.aeb_hold_request_status, type);
  convert(struct_v.aeb_risk_obj, ros_v.aeb_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::AEBPOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.aebp_state, ros_v.aebp_state, type);
  convert(struct_v.aebp_deceleration_requestStatus, ros_v.aebp_deceleration_requestStatus, type);
  convert(struct_v.aebp_deceleration_requestValue, ros_v.aebp_deceleration_requestValue, type);
  convert(struct_v.aebp_hold_request_status, ros_v.aebp_hold_request_status, type);
  convert(struct_v.aebp_risk_obj, ros_v.aebp_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::BSDOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.bsd_state, ros_v.bsd_state, type);
  convert(struct_v.bsd_left_warning_level, ros_v.bsd_left_warning_level, type);
  convert(struct_v.bsd_right_warning_level, ros_v.bsd_right_warning_level, type);
}

template <typename T2>
void convert(iflyauto::LCAOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lca_state, ros_v.lca_state, type);
  convert(struct_v.lca_left_warning_level, ros_v.lca_left_warning_level, type);
  convert(struct_v.lca_right_warning_level, ros_v.lca_right_warning_level, type);
}

template <typename T2>
void convert(iflyauto::DOWOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.dow_state, ros_v.dow_state, type);
  convert(struct_v.dow_front_left_warning_level, ros_v.dow_front_left_warning_level, type);
  convert(struct_v.dow_front_right_warning_level, ros_v.dow_front_right_warning_level, type);
  convert(struct_v.dow_rear_left_warning_level, ros_v.dow_rear_left_warning_level, type);
  convert(struct_v.dow_rear_right_warning_level, ros_v.dow_rear_right_warning_level, type);
}

template <typename T2>
void convert(iflyauto::RCWOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rcw_state, ros_v.rcw_state, type);
  convert(struct_v.rcw_warning_level, ros_v.rcw_warning_level, type);
}

template <typename T2>
void convert(iflyauto::FCTAOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fcta_state, ros_v.fcta_state, type);
  convert(struct_v.fcta_left_warning_level, ros_v.fcta_left_warning_level, type);
  convert(struct_v.fcta_right_warning_level, ros_v.fcta_right_warning_level, type);
  convert(struct_v.fcta_risk_obj, ros_v.fcta_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::RCTAOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rcta_state, ros_v.rcta_state, type);
  convert(struct_v.rcta_left_warning_level, ros_v.rcta_left_warning_level, type);
  convert(struct_v.rcta_right_warning_level, ros_v.rcta_right_warning_level, type);
  convert(struct_v.rcta_risk_obj, ros_v.rcta_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::FCTBOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.fctb_state, ros_v.fctb_state, type);
  convert(struct_v.fctb_deceleration_request_status, ros_v.fctb_deceleration_request_status, type);
  convert(struct_v.fctb_deceleration_request_value, ros_v.fctb_deceleration_request_value, type);
  convert(struct_v.fctb_hold_request_status, ros_v.fctb_hold_request_status, type);
  convert(struct_v.fctb_risk_obj, ros_v.fctb_risk_obj, type);
  convert(struct_v.fctb_active_source, ros_v.fctb_active_source, type);
}

template <typename T2>
void convert(iflyauto::RCTBOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rctb_state, ros_v.rctb_state, type);
  convert(struct_v.rctb_deceleration_request_status, ros_v.rctb_deceleration_request_status, type);
  convert(struct_v.rctb_deceleration_request_value, ros_v.rctb_deceleration_request_value, type);
  convert(struct_v.rctb_hold_request_status, ros_v.rctb_hold_request_status, type);
  convert(struct_v.rctb_risk_obj, ros_v.rctb_risk_obj, type);
  convert(struct_v.rctb_active_source, ros_v.rctb_active_source, type);
}

template <typename T2>
void convert(iflyauto::DAIOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.dai_state, ros_v.dai_state, type);
  convert(struct_v.dai_warning_request, ros_v.dai_warning_request, type);
}

template <typename T2>
void convert(iflyauto::ADASFunctionOutputInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
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
  convert(struct_v.dai_output_info, ros_v.dai_output_info, type);
}

