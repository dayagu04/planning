#pragma once

#include "base_convert.h"
#include "c/uss_wave_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::APASnsDtdObjDisInfoType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_sns_dtd_obj_first, ros_v.apa_sns_dtd_obj_first, type);
  convert(struct_v.obj1_type, ros_v.obj1_type, type);
  convert(struct_v.apa_sns_dtd_obj_sencond, ros_v.apa_sns_dtd_obj_sencond, type);
  convert(struct_v.obj2_type, ros_v.obj2_type, type);
  convert(struct_v.apa_sns_dtd_obj_three, ros_v.apa_sns_dtd_obj_three, type);
  convert(struct_v.obj3_type, ros_v.obj3_type, type);
}

template <typename T2>
void convert(iflyauto::APASnsDisProcessType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.apa_sns_dis_buf.size(); i0++) {
	  convert(struct_v.apa_sns_dis_buf[i0], ros_v.apa_sns_dis_buf[i0], type);
  }
  convert(struct_v.apa_sns_dtd_obj_dis_info_buf_read_index, ros_v.apa_sns_dtd_obj_dis_info_buf_read_index, type);
  convert(struct_v.apa_sns_dtd_obj_dis_info_buf_write_index, ros_v.apa_sns_dtd_obj_dis_info_buf_write_index, type);
  convert(struct_v.apa_sns_dtd_obj_dis_info_counter, ros_v.apa_sns_dtd_obj_dis_info_counter, type);
}

template <typename T2>
void convert(iflyauto::DisValueType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.wdis_value.size(); i0++) {
	  convert(struct_v.wdis_value[i0], ros_v.wdis_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::TypeValueType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.wtype_value.size(); i0++) {
	  convert(struct_v.wtype_value[i0], ros_v.wtype_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::XValueType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.wx_value.size(); i0++) {
	  convert(struct_v.wx_value[i0], ros_v.wx_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::YValueType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.wy_value.size(); i0++) {
	  convert(struct_v.wy_value[i0], ros_v.wy_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::DtObjUPASnsDtObjDisInfoType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.wdis.size(); i0++) {
	  convert(struct_v.wdis[i0], ros_v.wdis[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.wtype.size(); i1++) {
	  convert(struct_v.wtype[i1], ros_v.wtype[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.wx.size(); i2++) {
	  convert(struct_v.wx[i2], ros_v.wx[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.wy.size(); i3++) {
	  convert(struct_v.wy[i3], ros_v.wy[i3], type);
  }
}

template <typename T2>
void convert(iflyauto::UssWaveInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.uss_state.size(); i0++) {
	  convert(struct_v.uss_state[i0], ros_v.uss_state[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.upa_ori_dis_buffer.size(); i1++) {
	  convert(struct_v.upa_ori_dis_buffer[i1], ros_v.upa_ori_dis_buffer[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.apa_ori_dis_buffer.size(); i2++) {
	  convert(struct_v.apa_ori_dis_buffer[i2], ros_v.apa_ori_dis_buffer[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.apa_dis_info_buf.size(); i3++) {
	  convert(struct_v.apa_dis_info_buf[i3], ros_v.apa_dis_info_buf[i3], type);
  }
  for (size_t i4 = 0; i4 < ros_v.upa_dis_info_buf.size(); i4++) {
	  convert(struct_v.upa_dis_info_buf[i4], ros_v.upa_dis_info_buf[i4], type);
  }
}

template <typename T2>
void convert(iflyauto::UssPdcPasSonarDistanceType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pas_sonarx_distance, ros_v.pas_sonarx_distance, type);
  convert(struct_v.pas_sonarx_blind, ros_v.pas_sonarx_blind, type);
  convert(struct_v.pas_sonarx_ditance_time, ros_v.pas_sonarx_ditance_time, type);
  convert(struct_v.pas_sonarx_cross_distance_left, ros_v.pas_sonarx_cross_distance_left, type);
  convert(struct_v.pas_sonarx_cross_distance_right, ros_v.pas_sonarx_cross_distance_right, type);
  convert(struct_v.pas_sonarx_cross_ditance_left_time, ros_v.pas_sonarx_cross_ditance_left_time, type);
  convert(struct_v.pas_sonarx_cross_ditance_right_time, ros_v.pas_sonarx_cross_ditance_right_time, type);
  convert(struct_v.pas_sonarx_confidence, ros_v.pas_sonarx_confidence, type);
  convert(struct_v.pas_sonarx_counter, ros_v.pas_sonarx_counter, type);
  convert(struct_v.pas_sonarx_tof1_distance, ros_v.pas_sonarx_tof1_distance, type);
  convert(struct_v.pas_sonarx_tof2_distance, ros_v.pas_sonarx_tof2_distance, type);
  convert(struct_v.pas_sonarx_ringing_time, ros_v.pas_sonarx_ringing_time, type);
  convert(struct_v.pas_sonarx_echo_tof1, ros_v.pas_sonarx_echo_tof1, type);
  convert(struct_v.pas_sonarx_echo_width1, ros_v.pas_sonarx_echo_width1, type);
  convert(struct_v.pas_sonarx_echo_tof2, ros_v.pas_sonarx_echo_tof2, type);
  convert(struct_v.pas_sonarx_echo_width2, ros_v.pas_sonarx_echo_width2, type);
  convert(struct_v.pas_sonarx_echo_peak1, ros_v.pas_sonarx_echo_peak1, type);
  convert(struct_v.pas_sonarx_echo_peak2, ros_v.pas_sonarx_echo_peak2, type);
  convert(struct_v.pas_sonarx_emit, ros_v.pas_sonarx_emit, type);
}

template <typename T2>
void convert(iflyauto::UssPdcPrivPointDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.point_id, ros_v.point_id, type);
  convert(struct_v.point_valid, ros_v.point_valid, type);
  convert(struct_v.point_x, ros_v.point_x, type);
  convert(struct_v.point_y, ros_v.point_y, type);
  convert(struct_v.point_error_a, ros_v.point_error_a, type);
  convert(struct_v.point_error_b, ros_v.point_error_b, type);
  convert(struct_v.point_error_theta, ros_v.point_error_theta, type);
  convert(struct_v.point_high, ros_v.point_high, type);
  convert(struct_v.point_exist_probability, ros_v.point_exist_probability, type);
}

template <typename T2>
void convert(iflyauto::UssPdcPrivPointType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.point_priv_timestamp.size(); i0++) {
	  convert(struct_v.point_priv_timestamp[i0], ros_v.point_priv_timestamp[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.priv_point_data_prop.size(); i1++) {
	  convert(struct_v.priv_point_data_prop[i1], ros_v.priv_point_data_prop[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::UssPdcIccSendDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.sonar_distance_data.size(); i0++) {
	  convert(struct_v.sonar_distance_data[i0], ros_v.sonar_distance_data[i0], type);
  }
  convert(struct_v.priv_point_data, ros_v.priv_point_data, type);
}

template <typename T2>
void convert(iflyauto::UssPdcUpcVehiclePosDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.upc_vehicle_position_x, ros_v.upc_vehicle_position_x, type);
  convert(struct_v.upc_vehicle_position_y, ros_v.upc_vehicle_position_y, type);
  convert(struct_v.upc_vehicle_position_update_time, ros_v.upc_vehicle_position_update_time, type);
  convert(struct_v.upc_vehicle_position_curvature, ros_v.upc_vehicle_position_curvature, type);
  convert(struct_v.upc_vechile_position_yaw_angle, ros_v.upc_vechile_position_yaw_angle, type);
  convert(struct_v.upc_vechile_position_drive_distance, ros_v.upc_vechile_position_drive_distance, type);
}

template <typename T2>
void convert(iflyauto::UssPdcIccRecvDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.upc_vehicle_pos_data, ros_v.upc_vehicle_pos_data, type);
}

