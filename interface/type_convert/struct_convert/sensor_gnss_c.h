#pragma once

#include "base_convert.h"
#include "c/sensor_gnss_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::IFLYGnssSatelliteInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.gnss_type, ros_v.gnss_type, type);
  convert(struct_v.gnss_num_in_view, ros_v.gnss_num_in_view, type);
  convert(struct_v.gnss_sat_prn, ros_v.gnss_sat_prn, type);
  convert(struct_v.gnss_sat_snr, ros_v.gnss_sat_snr, type);
  convert(struct_v.gnss_sat_elev, ros_v.gnss_sat_elev, type);
  convert(struct_v.gnss_sat_azimuth, ros_v.gnss_sat_azimuth, type);
}

template <typename T2>
void convert(iflyauto::IFLYGnssMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.gnss_altitude, ros_v.gnss_altitude, type);
  convert(struct_v.gnss_ellipsoid, ros_v.gnss_ellipsoid, type);
  convert(struct_v.gnss_source, ros_v.gnss_source, type);
  convert(struct_v.gnss_llh_type, ros_v.gnss_llh_type, type);
  convert(struct_v.gnss_quality, ros_v.gnss_quality, type);
  convert(struct_v.gnss_pos_status, ros_v.gnss_pos_status, type);
  convert(struct_v.gnss_num_satellites, ros_v.gnss_num_satellites, type);
  convert(struct_v.gnss_tdop, ros_v.gnss_tdop, type);
  convert(struct_v.gnss_hdop, ros_v.gnss_hdop, type);
  convert(struct_v.gnss_vdop, ros_v.gnss_vdop, type);
  convert(struct_v.gnss_heading, ros_v.gnss_heading, type);
  convert(struct_v.gnss_course, ros_v.gnss_course, type);
  convert(struct_v.gnss_heading_err, ros_v.gnss_heading_err, type);
  convert(struct_v.gnss_course_err, ros_v.gnss_course_err, type);
  convert(struct_v.gnss_lat, ros_v.gnss_lat, type);
  convert(struct_v.gnss_lon, ros_v.gnss_lon, type);
  convert(struct_v.gnss_horipos_err, ros_v.gnss_horipos_err, type);
  convert(struct_v.gnss_vertpos_err, ros_v.gnss_vertpos_err, type);
  convert(struct_v.gnss_horivel_err, ros_v.gnss_horivel_err, type);
  convert(struct_v.gnss_vertvel_err, ros_v.gnss_vertvel_err, type);
  convert(struct_v.gnss_vel_north, ros_v.gnss_vel_north, type);
  convert(struct_v.gnss_vel_east, ros_v.gnss_vel_east, type);
  convert(struct_v.gnss_vel_down, ros_v.gnss_vel_down, type);
}

template <typename T2>
void convert(iflyauto::IFLYInsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.angle_heading, ros_v.angle_heading, type);
  convert(struct_v.angle_pitch, ros_v.angle_pitch, type);
  convert(struct_v.angle_roll, ros_v.angle_roll, type);
}

template <typename T2>
void convert(iflyauto::IFLYGnss &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.gnss_name.size(); i0++) {
	  convert(struct_v.gnss_name[i0], ros_v.gnss_name[i0], type);
  }
  convert(struct_v.gnss_msg, ros_v.gnss_msg, type);
  for (size_t i1 = 0; i1 < ros_v.gnss_satellite_info.size(); i1++) {
	  convert(struct_v.gnss_satellite_info[i1], ros_v.gnss_satellite_info[i1], type);
  }
  convert(struct_v.ins_info, ros_v.ins_info, type);
}

