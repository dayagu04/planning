#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AbsolutePostion.h"
#include "struct_msgs_v2_10/AbsolutePostion.h"
#include "struct_msgs/LaneGroupInRoute.h"
#include "struct_msgs_v2_10/LaneGroupInRoute.h"
#include "struct_msgs/LaneInRoute.h"
#include "struct_msgs_v2_10/LaneInRoute.h"
#include "struct_msgs/Position.h"
#include "struct_msgs_v2_10/Position.h"
#include "struct_msgs/PositionGeofence.h"
#include "struct_msgs_v2_10/PositionGeofence.h"
#include "struct_msgs/PositionWarning.h"
#include "struct_msgs_v2_10/PositionWarning.h"
#include "struct_msgs/PostionData.h"
#include "struct_msgs_v2_10/PostionData.h"
#include "struct_msgs/PostionFailSafe.h"
#include "struct_msgs_v2_10/PostionFailSafe.h"
#include "struct_msgs/RelativePostion.h"
#include "struct_msgs_v2_10/RelativePostion.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AbsolutePostion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.original_loc_timestamp, ros_v.original_loc_timestamp, type);
	convert(old_ros_v.lon, ros_v.lon, type);
	convert(old_ros_v.lat, ros_v.lat, type);
	convert(old_ros_v.heading, ros_v.heading, type);
	convert(old_ros_v.velocity_x, ros_v.velocity_x, type);
	convert(old_ros_v.velocity_y, ros_v.velocity_y, type);
	convert(old_ros_v.velocity_z, ros_v.velocity_z, type);
	convert(old_ros_v.x_acc, ros_v.x_acc, type);
	convert(old_ros_v.y_acc, ros_v.y_acc, type);
	convert(old_ros_v.z_acc, ros_v.z_acc, type);
	convert(old_ros_v.angular_velocity_x, ros_v.angular_velocity_x, type);
	convert(old_ros_v.angular_velocity_y, ros_v.angular_velocity_y, type);
	convert(old_ros_v.angular_velocity_z, ros_v.angular_velocity_z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneGroupInRoute &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_group_id, ros_v.lane_group_id, type);
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.lanes_in_route_size, ros_v.lanes_in_route_size, type);
	ros_v.lanes_in_route.resize(old_ros_v.lanes_in_route.size());
	for (int i = 0; i < ros_v.lanes_in_route.size(); i++) {
	    convert(old_ros_v.lanes_in_route[i], ros_v.lanes_in_route[i], type);
	}
	convert(old_ros_v.start_point, ros_v.start_point, type);
	convert(old_ros_v.end_point, ros_v.end_point, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneInRoute &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_id, ros_v.lane_id, type);
	convert(old_ros_v.entry_lane_ids_size, ros_v.entry_lane_ids_size, type);
	ros_v.entry_lane_ids.resize(old_ros_v.entry_lane_ids.size());
	for (int i = 0; i < ros_v.entry_lane_ids.size(); i++) {
	    convert(old_ros_v.entry_lane_ids[i], ros_v.entry_lane_ids[i], type);
	}
	convert(old_ros_v.exit_lane_ids_size, ros_v.exit_lane_ids_size, type);
	ros_v.exit_lane_ids.resize(old_ros_v.exit_lane_ids.size());
	for (int i = 0; i < ros_v.exit_lane_ids.size(); i++) {
	    convert(old_ros_v.exit_lane_ids[i], ros_v.exit_lane_ids[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Position &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.position_age, ros_v.position_age, type);
	convert(old_ros_v.positions_size, ros_v.positions_size, type);
	ros_v.positions.resize(old_ros_v.positions.size());
	for (int i = 0; i < ros_v.positions.size(); i++) {
	    convert(old_ros_v.positions[i], ros_v.positions[i], type);
	}
	convert(old_ros_v.relative_pos_size, ros_v.relative_pos_size, type);
	ros_v.relative_pos.resize(old_ros_v.relative_pos.size());
	for (int i = 0; i < ros_v.relative_pos.size(); i++) {
	    convert(old_ros_v.relative_pos[i], ros_v.relative_pos[i], type);
	}
	convert(old_ros_v.absolute_pos_size, ros_v.absolute_pos_size, type);
	ros_v.absolute_pos.resize(old_ros_v.absolute_pos.size());
	for (int i = 0; i < ros_v.absolute_pos.size(); i++) {
	    convert(old_ros_v.absolute_pos[i], ros_v.absolute_pos[i], type);
	}
	convert(old_ros_v.position_warning_size, ros_v.position_warning_size, type);
	ros_v.position_warning.resize(old_ros_v.position_warning.size());
	for (int i = 0; i < ros_v.position_warning.size(); i++) {
	    convert(old_ros_v.position_warning[i], ros_v.position_warning[i], type);
	}
	convert(old_ros_v.position_geofence_size, ros_v.position_geofence_size, type);
	ros_v.position_geofence.resize(old_ros_v.position_geofence.size());
	for (int i = 0; i < ros_v.position_geofence.size(); i++) {
	    convert(old_ros_v.position_geofence[i], ros_v.position_geofence[i], type);
	}
	convert(old_ros_v.fail_safe_size, ros_v.fail_safe_size, type);
	ros_v.fail_safe.resize(old_ros_v.fail_safe.size());
	for (int i = 0; i < ros_v.fail_safe.size(); i++) {
	    convert(old_ros_v.fail_safe[i], ros_v.fail_safe[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PositionGeofence &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.geofence_judge_status, ros_v.geofence_judge_status, type);
	convert(old_ros_v.geofence_judge_type, ros_v.geofence_judge_type, type);
	convert(old_ros_v.geofence_bounding_distance, ros_v.geofence_bounding_distance, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PositionWarning &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.warning_judge_status, ros_v.warning_judge_status, type);
	convert(old_ros_v.warning_judge_type, ros_v.warning_judge_type, type);
	convert(old_ros_v.warning_bounding_distance, ros_v.warning_bounding_distance, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostionData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.path_id, ros_v.path_id, type);
	convert(old_ros_v.offset, ros_v.offset, type);
	convert(old_ros_v.accuracy, ros_v.accuracy, type);
	convert(old_ros_v.lateral_accuracy, ros_v.lateral_accuracy, type);
	convert(old_ros_v.longitudinal_accuracy, ros_v.longitudinal_accuracy, type);
	convert(old_ros_v.original_lon, ros_v.original_lon, type);
	convert(old_ros_v.original_lat, ros_v.original_lat, type);
	convert(old_ros_v.deviation, ros_v.deviation, type);
	convert(old_ros_v.vehicle_speed, ros_v.vehicle_speed, type);
	convert(old_ros_v.relative_heading, ros_v.relative_heading, type);
	convert(old_ros_v.probability, ros_v.probability, type);
	convert(old_ros_v.current_lane, ros_v.current_lane, type);
	convert(old_ros_v.preferred_path, ros_v.preferred_path, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PostionFailSafe &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.failsafe_loc_status, ros_v.failsafe_loc_status, type);
	convert(old_ros_v.failsafe_gnss_status, ros_v.failsafe_gnss_status, type);
	convert(old_ros_v.failsafe_camera_status, ros_v.failsafe_camera_status, type);
	convert(old_ros_v.failsafe_hdmap_status, ros_v.failsafe_hdmap_status, type);
	convert(old_ros_v.failsafe_vehicle_status, ros_v.failsafe_vehicle_status, type);
	convert(old_ros_v.failsafe_imu_status, ros_v.failsafe_imu_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RelativePostion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.road_id, ros_v.road_id, type);
	convert(old_ros_v.lane_id, ros_v.lane_id, type);
	convert(old_ros_v.lane_seq, ros_v.lane_seq, type);
	convert(old_ros_v.dis_left, ros_v.dis_left, type);
	convert(old_ros_v.dis_right, ros_v.dis_right, type);
	convert(old_ros_v.head_left, ros_v.head_left, type);
	convert(old_ros_v.head_right, ros_v.head_right, type);
	convert(old_ros_v.confi_lane_loc, ros_v.confi_lane_loc, type);
}

REG_CONVERT_SINGLE(_iflytek_ehr_position_converter, "/iflytek/ehr/position", Position);
