#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/IFLYGnss.h"
#include "struct_msgs_v2_10/IFLYGnss.h"
#include "struct_msgs/IFLYGnssMsg.h"
#include "struct_msgs_v2_10/IFLYGnssMsg.h"
#include "struct_msgs/IFLYGnssSatelliteInfo.h"
#include "struct_msgs_v2_10/IFLYGnssSatelliteInfo.h"
#include "struct_msgs/IFLYInsInfo.h"
#include "struct_msgs_v2_10/IFLYInsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYGnss &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.gnss_name[i], ros_v.gnss_name[i], type);
	}
	convert(old_ros_v.gnss_msg, ros_v.gnss_msg, type);
	for (int i = 0; i < 20; i++) {
	    convert(old_ros_v.gnss_satellite_info[i], ros_v.gnss_satellite_info[i], type);
	}
	convert(old_ros_v.ins_info, ros_v.ins_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYGnssMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.gnss_altitude, ros_v.gnss_altitude, type);
	convert(old_ros_v.gnss_ellipsoid, ros_v.gnss_ellipsoid, type);
	convert(old_ros_v.gnss_quality, ros_v.gnss_quality, type);
	convert(old_ros_v.gnss_pos_status, ros_v.gnss_pos_status, type);
	convert(old_ros_v.gnss_num_satellites, ros_v.gnss_num_satellites, type);
	convert(old_ros_v.gnss_tdop, ros_v.gnss_tdop, type);
	convert(old_ros_v.gnss_hdop, ros_v.gnss_hdop, type);
	convert(old_ros_v.gnss_vdop, ros_v.gnss_vdop, type);
	convert(old_ros_v.gnss_heading, ros_v.gnss_heading, type);
	convert(old_ros_v.gnss_course, ros_v.gnss_course, type);
	convert(old_ros_v.gnss_heading_err, ros_v.gnss_heading_err, type);
	convert(old_ros_v.gnss_course_err, ros_v.gnss_course_err, type);
	convert(old_ros_v.gnss_lat, ros_v.gnss_lat, type);
	convert(old_ros_v.gnss_lon, ros_v.gnss_lon, type);
	convert(old_ros_v.gnss_horipos_err, ros_v.gnss_horipos_err, type);
	convert(old_ros_v.gnss_vertpos_err, ros_v.gnss_vertpos_err, type);
	convert(old_ros_v.gnss_horivel_err, ros_v.gnss_horivel_err, type);
	convert(old_ros_v.gnss_vertvel_err, ros_v.gnss_vertvel_err, type);
	convert(old_ros_v.gnss_vel_north, ros_v.gnss_vel_north, type);
	convert(old_ros_v.gnss_vel_east, ros_v.gnss_vel_east, type);
	convert(old_ros_v.gnss_vel_down, ros_v.gnss_vel_down, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYGnssSatelliteInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.gnss_type, ros_v.gnss_type, type);
	convert(old_ros_v.gnss_num_in_view, ros_v.gnss_num_in_view, type);
	convert(old_ros_v.gnss_sat_prn, ros_v.gnss_sat_prn, type);
	convert(old_ros_v.gnss_sat_snr, ros_v.gnss_sat_snr, type);
	convert(old_ros_v.gnss_sat_elev, ros_v.gnss_sat_elev, type);
	convert(old_ros_v.gnss_sat_azimuth, ros_v.gnss_sat_azimuth, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYInsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.angle_heading, ros_v.angle_heading, type);
	convert(old_ros_v.angle_pitch, ros_v.angle_pitch, type);
	convert(old_ros_v.angle_roll, ros_v.angle_roll, type);
}

REG_CONVERT_SINGLE(_iflytek_sensor_gnss_converter, "/iflytek/sensor/gnss", IFLYGnss);
REG_CONVERT_SINGLE(_iflytek_sensor_gnss_tbox_converter, "/iflytek/sensor/gnss_tbox", IFLYGnss);
