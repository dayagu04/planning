#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ADASFunctionOutputInfo.h"
#include "struct_msgs_v2_10/ADASFunctionOutputInfo.h"
#include "struct_msgs/AEBOutputInfo.h"
#include "struct_msgs_v2_10/AEBOutputInfo.h"
#include "struct_msgs/AEBPOutputInfo.h"
#include "struct_msgs_v2_10/AEBPOutputInfo.h"
#include "struct_msgs/BSDOutputInfo.h"
#include "struct_msgs_v2_10/BSDOutputInfo.h"
#include "struct_msgs/DOWOutputInfo.h"
#include "struct_msgs_v2_10/DOWOutputInfo.h"
#include "struct_msgs/FCTAOutputInfo.h"
#include "struct_msgs_v2_10/FCTAOutputInfo.h"
#include "struct_msgs/FCTBOutputInfo.h"
#include "struct_msgs_v2_10/FCTBOutputInfo.h"
#include "struct_msgs/FCWOutputInfo.h"
#include "struct_msgs_v2_10/FCWOutputInfo.h"
#include "struct_msgs/LCAOutputInfo.h"
#include "struct_msgs_v2_10/LCAOutputInfo.h"
#include "struct_msgs/PCWOutputInfo.h"
#include "struct_msgs_v2_10/PCWOutputInfo.h"
#include "struct_msgs/RCTAOutputInfo.h"
#include "struct_msgs_v2_10/RCTAOutputInfo.h"
#include "struct_msgs/RCTBOutputInfo.h"
#include "struct_msgs_v2_10/RCTBOutputInfo.h"
#include "struct_msgs/RCWOutputInfo.h"
#include "struct_msgs_v2_10/RCWOutputInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ADASFunctionOutputInfo &ros_v, ConvertTypeInfo type) {
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
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AEBOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.aeb_state, ros_v.aeb_state, type);
	convert(old_ros_v.aeb_deceleration_request_status, ros_v.aeb_deceleration_request_status, type);
	convert(old_ros_v.aeb_deceleration_request_value, ros_v.aeb_deceleration_request_value, type);
	convert(old_ros_v.aeb_hold_request_status, ros_v.aeb_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AEBPOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.aebp_state, ros_v.aebp_state, type);
	convert(old_ros_v.aebp_deceleration_requestStatus, ros_v.aebp_deceleration_requestStatus, type);
	convert(old_ros_v.aebp_deceleration_requestValue, ros_v.aebp_deceleration_requestValue, type);
	convert(old_ros_v.aebp_hold_request_status, ros_v.aebp_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::BSDOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.bsd_state, ros_v.bsd_state, type);
	convert(old_ros_v.bsd_left_warning_level, ros_v.bsd_left_warning_level, type);
	convert(old_ros_v.bsd_right_warning_level, ros_v.bsd_right_warning_level, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::DOWOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.dow_state, ros_v.dow_state, type);
	convert(old_ros_v.dow_front_left_warning_level, ros_v.dow_front_left_warning_level, type);
	convert(old_ros_v.dow_front_right_warning_level, ros_v.dow_front_right_warning_level, type);
	convert(old_ros_v.dow_rear_left_warning_level, ros_v.dow_rear_left_warning_level, type);
	convert(old_ros_v.dow_rear_right_warning_level, ros_v.dow_rear_right_warning_level, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCTAOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fcta_state, ros_v.fcta_state, type);
	convert(old_ros_v.fcta_left_warning_level, ros_v.fcta_left_warning_level, type);
	convert(old_ros_v.fcta_right_warning_level, ros_v.fcta_right_warning_level, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCTBOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fctb_state, ros_v.fctb_state, type);
	convert(old_ros_v.fctb_deceleration_request_status, ros_v.fctb_deceleration_request_status, type);
	convert(old_ros_v.fctb_deceleration_request_value, ros_v.fctb_deceleration_request_value, type);
	convert(old_ros_v.fctb_hold_request_status, ros_v.fctb_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FCWOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.fcw_state, ros_v.fcw_state, type);
	convert(old_ros_v.fcw_warning_level, ros_v.fcw_warning_level, type);
	convert(old_ros_v.fcw_Prefill_request_status, ros_v.fcw_Prefill_request_status, type);
	convert(old_ros_v.fcw_jerk_request_status, ros_v.fcw_jerk_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LCAOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lca_state, ros_v.lca_state, type);
	convert(old_ros_v.lca_left_warning_level, ros_v.lca_left_warning_level, type);
	convert(old_ros_v.lca_right_warning_level, ros_v.lca_right_warning_level, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PCWOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.pcw_state, ros_v.pcw_state, type);
	convert(old_ros_v.pcw_warning_level, ros_v.pcw_warning_level, type);
	convert(old_ros_v.pcw_prefill_request_status, ros_v.pcw_prefill_request_status, type);
	convert(old_ros_v.pcw_jerk_request_status, ros_v.pcw_jerk_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCTAOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rcta_state, ros_v.rcta_state, type);
	convert(old_ros_v.rcta_left_warning_level, ros_v.rcta_left_warning_level, type);
	convert(old_ros_v.rcta_right_warning_level, ros_v.rcta_right_warning_level, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCTBOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rctb_state, ros_v.rctb_state, type);
	convert(old_ros_v.rctb_deceleration_request_status, ros_v.rctb_deceleration_request_status, type);
	convert(old_ros_v.rctb_deceleration_request_value, ros_v.rctb_deceleration_request_value, type);
	convert(old_ros_v.rctb_hold_request_status, ros_v.rctb_hold_request_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RCWOutputInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rcw_state, ros_v.rcw_state, type);
	convert(old_ros_v.rcw_warning_level, ros_v.rcw_warning_level, type);
}

REG_CONVERT_SINGLE(_iflytek_adas_function_adas_converter, "/iflytek/adas_function/adas", ADASFunctionOutputInfo);
