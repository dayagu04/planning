#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/RadarPerceptionObject.h"
#include "struct_msgs_v2_10/RadarPerceptionObject.h"
#include "struct_msgs/RadarPerceptionObjectsInfo.h"
#include "struct_msgs_v2_10/RadarPerceptionObjectsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RadarPerceptionObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.shape, ros_v.shape, type);
	convert(old_ros_v.relative_position, ros_v.relative_position, type);
	convert(old_ros_v.relative_velocity, ros_v.relative_velocity, type);
	convert(old_ros_v.relative_acceleration, ros_v.relative_acceleration, type);
	convert(old_ros_v.relative_heading_angle, ros_v.relative_heading_angle, type);
	convert(old_ros_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
	convert(old_ros_v.age, ros_v.age, type);
	convert(old_ros_v.conf, ros_v.conf, type);
	convert(old_ros_v.orientation_angle_conf, ros_v.orientation_angle_conf, type);
	convert(old_ros_v.vel_conf, ros_v.vel_conf, type);
	convert(old_ros_v.accel_conf, ros_v.accel_conf, type);
	convert(old_ros_v.obj_update_flag, ros_v.obj_update_flag, type);
	convert(old_ros_v.obj_motion_pattern, ros_v.obj_motion_pattern, type);
	convert(old_ros_v.obstacle_prob, ros_v.obstacle_prob, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RadarPerceptionObjectsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	convert(old_ros_v.sensor_type, ros_v.sensor_type, type);
	convert(old_ros_v.object_list_size, ros_v.object_list_size, type);
	ros_v.object_list.resize(old_ros_v.object_list.size());
	for (int i = 0; i < ros_v.object_list.size(); i++) {
	    convert(old_ros_v.object_list[i], ros_v.object_list[i], type);
	}
	// warning : added struct member : uint32 crc
}

REG_CONVERT_SINGLE(_iflytek_radar_fm_perception_4d_target_converter, "/iflytek/radar_fm_perception/4d_target", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_fm_perception_4d_obstacle_converter, "/iflytek/radar_fm_perception/4d_obstacle", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_fm_perception_info_converter, "/iflytek/radar_fm_perception_info", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_fl_perception_info_converter, "/iflytek/radar_fl_perception_info", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_fr_perception_info_converter, "/iflytek/radar_fr_perception_info", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_rl_perception_info_converter, "/iflytek/radar_rl_perception_info", RadarPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_radar_rr_perception_info_converter, "/iflytek/radar_rr_perception_info", RadarPerceptionObjectsInfo);
