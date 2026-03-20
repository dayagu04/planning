#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/APASnsDisProcessType.h"
#include "struct_msgs_v2_10/APASnsDisProcessType.h"
#include "struct_msgs/APASnsDtdObjDisInfoType.h"
#include "struct_msgs_v2_10/APASnsDtdObjDisInfoType.h"
#include "struct_msgs/DisValueType.h"
#include "struct_msgs_v2_10/DisValueType.h"
#include "struct_msgs/DtObjUPASnsDtObjDisInfoType.h"
#include "struct_msgs_v2_10/DtObjUPASnsDtObjDisInfoType.h"
#include "struct_msgs/TypeValueType.h"
#include "struct_msgs_v2_10/TypeValueType.h"
#include "struct_msgs/UssPdcIccRecvDataType.h"
#include "struct_msgs_v2_10/UssPdcIccRecvDataType.h"
#include "struct_msgs/UssPdcIccSendDataType.h"
#include "struct_msgs_v2_10/UssPdcIccSendDataType.h"
#include "struct_msgs/UssPdcPasSonarDistanceType.h"
#include "struct_msgs_v2_10/UssPdcPasSonarDistanceType.h"
#include "struct_msgs/UssPdcPrivPointDataType.h"
#include "struct_msgs_v2_10/UssPdcPrivPointDataType.h"
#include "struct_msgs/UssPdcPrivPointType.h"
#include "struct_msgs_v2_10/UssPdcPrivPointType.h"
#include "struct_msgs/UssPdcUpcVehiclePosDataType.h"
#include "struct_msgs_v2_10/UssPdcUpcVehiclePosDataType.h"
#include "struct_msgs/UssWaveInfo.h"
#include "struct_msgs_v2_10/UssWaveInfo.h"
#include "struct_msgs/XValueType.h"
#include "struct_msgs_v2_10/XValueType.h"
#include "struct_msgs/YValueType.h"
#include "struct_msgs_v2_10/YValueType.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::APASnsDisProcessType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 5; i++) {
	    convert(old_ros_v.apa_sns_dis_buf[i], ros_v.apa_sns_dis_buf[i], type);
	}
	convert(old_ros_v.apa_sns_dtd_obj_dis_info_buf_read_index, ros_v.apa_sns_dtd_obj_dis_info_buf_read_index, type);
	convert(old_ros_v.apa_sns_dtd_obj_dis_info_buf_write_index, ros_v.apa_sns_dtd_obj_dis_info_buf_write_index, type);
	convert(old_ros_v.apa_sns_dtd_obj_dis_info_counter, ros_v.apa_sns_dtd_obj_dis_info_counter, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::APASnsDtdObjDisInfoType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.apa_sns_dtd_obj_first, ros_v.apa_sns_dtd_obj_first, type);
	convert(old_ros_v.obj1_type, ros_v.obj1_type, type);
	convert(old_ros_v.apa_sns_dtd_obj_sencond, ros_v.apa_sns_dtd_obj_sencond, type);
	convert(old_ros_v.obj2_type, ros_v.obj2_type, type);
	convert(old_ros_v.apa_sns_dtd_obj_three, ros_v.apa_sns_dtd_obj_three, type);
	convert(old_ros_v.obj3_type, ros_v.obj3_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::DisValueType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 5; i++) {
	    convert(old_ros_v.wdis_value[i], ros_v.wdis_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::DtObjUPASnsDtObjDisInfoType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.wdis[i], ros_v.wdis[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.wtype[i], ros_v.wtype[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.wx[i], ros_v.wx[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.wy[i], ros_v.wy[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TypeValueType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 5; i++) {
	    convert(old_ros_v.wtype_value[i], ros_v.wtype_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcIccRecvDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.upc_vehicle_pos_data, ros_v.upc_vehicle_pos_data, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcIccSendDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sonar_distance_data[i], ros_v.sonar_distance_data[i], type);
	}
	convert(old_ros_v.priv_point_data, ros_v.priv_point_data, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcPasSonarDistanceType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.pas_sonarx_distance, ros_v.pas_sonarx_distance, type);
	convert(old_ros_v.pas_sonarx_blind, ros_v.pas_sonarx_blind, type);
	convert(old_ros_v.pas_sonarx_ditance_time, ros_v.pas_sonarx_ditance_time, type);
	convert(old_ros_v.pas_sonarx_cross_distance_left, ros_v.pas_sonarx_cross_distance_left, type);
	convert(old_ros_v.pas_sonarx_cross_distance_right, ros_v.pas_sonarx_cross_distance_right, type);
	convert(old_ros_v.pas_sonarx_cross_ditance_left_time, ros_v.pas_sonarx_cross_ditance_left_time, type);
	convert(old_ros_v.pas_sonarx_cross_ditance_right_time, ros_v.pas_sonarx_cross_ditance_right_time, type);
	convert(old_ros_v.pas_sonarx_confidence, ros_v.pas_sonarx_confidence, type);
	convert(old_ros_v.pas_sonarx_counter, ros_v.pas_sonarx_counter, type);
	convert(old_ros_v.pas_sonarx_tof1_distance, ros_v.pas_sonarx_tof1_distance, type);
	convert(old_ros_v.pas_sonarx_tof2_distance, ros_v.pas_sonarx_tof2_distance, type);
	convert(old_ros_v.pas_sonarx_ringing_time, ros_v.pas_sonarx_ringing_time, type);
	convert(old_ros_v.pas_sonarx_echo_tof1, ros_v.pas_sonarx_echo_tof1, type);
	convert(old_ros_v.pas_sonarx_echo_width1, ros_v.pas_sonarx_echo_width1, type);
	convert(old_ros_v.pas_sonarx_echo_tof2, ros_v.pas_sonarx_echo_tof2, type);
	convert(old_ros_v.pas_sonarx_echo_width2, ros_v.pas_sonarx_echo_width2, type);
	convert(old_ros_v.pas_sonarx_echo_peak1, ros_v.pas_sonarx_echo_peak1, type);
	convert(old_ros_v.pas_sonarx_echo_peak2, ros_v.pas_sonarx_echo_peak2, type);
	convert(old_ros_v.pas_sonarx_emit, ros_v.pas_sonarx_emit, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcPrivPointDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.point_id, ros_v.point_id, type);
	convert(old_ros_v.point_valid, ros_v.point_valid, type);
	convert(old_ros_v.point_x, ros_v.point_x, type);
	convert(old_ros_v.point_y, ros_v.point_y, type);
	convert(old_ros_v.point_error_a, ros_v.point_error_a, type);
	convert(old_ros_v.point_error_b, ros_v.point_error_b, type);
	convert(old_ros_v.point_error_theta, ros_v.point_error_theta, type);
	convert(old_ros_v.point_high, ros_v.point_high, type);
	convert(old_ros_v.point_exist_probability, ros_v.point_exist_probability, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcPrivPointType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 22; i++) {
	    convert(old_ros_v.point_priv_timestamp[i], ros_v.point_priv_timestamp[i], type);
	}
	for (int i = 0; i < 150; i++) {
	    convert(old_ros_v.priv_point_data_prop[i], ros_v.priv_point_data_prop[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssPdcUpcVehiclePosDataType &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.upc_vehicle_position_x, ros_v.upc_vehicle_position_x, type);
	convert(old_ros_v.upc_vehicle_position_y, ros_v.upc_vehicle_position_y, type);
	convert(old_ros_v.upc_vehicle_position_update_time, ros_v.upc_vehicle_position_update_time, type);
	convert(old_ros_v.upc_vehicle_position_curvature, ros_v.upc_vehicle_position_curvature, type);
	convert(old_ros_v.upc_vechile_position_yaw_angle, ros_v.upc_vechile_position_yaw_angle, type);
	convert(old_ros_v.upc_vechile_position_drive_distance, ros_v.upc_vechile_position_drive_distance, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssWaveInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.uss_state[i], ros_v.uss_state[i], type);
	}
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.upa_ori_dis_buffer[i], ros_v.upa_ori_dis_buffer[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.apa_ori_dis_buffer[i], ros_v.apa_ori_dis_buffer[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.apa_dis_info_buf[i], ros_v.apa_dis_info_buf[i], type);
	}
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.upa_dis_info_buf[i], ros_v.upa_dis_info_buf[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::XValueType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 5; i++) {
	    convert(old_ros_v.wx_value[i], ros_v.wx_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::YValueType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 5; i++) {
	    convert(old_ros_v.wy_value[i], ros_v.wy_value[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_uss_usswave_info_converter, "/iflytek/uss/usswave_info", UssPdcIccSendDataType);
