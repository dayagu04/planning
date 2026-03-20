#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/ControlOutput.h"
#include "struct_msgs_v2_10/ControlOutput.h"
#include "struct_msgs/ControlStatus.h"
#include "struct_msgs_v2_10/ControlStatus.h"
#include "struct_msgs/ControlTrajectory.h"
#include "struct_msgs_v2_10/ControlTrajectory.h"
#include "struct_msgs/LatControlMode.h"
#include "struct_msgs_v2_10/LatControlMode.h"
#include "struct_msgs/LonControlMode.h"
#include "struct_msgs_v2_10/LonControlMode.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ControlOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.lat_control_mode, ros_v.lat_control_mode, type);
	convert(old_ros_v.steering, ros_v.steering, type);
	convert(old_ros_v.steering_rate, ros_v.steering_rate, type);
	convert(old_ros_v.steering_torque, ros_v.steering_torque, type);
	convert(old_ros_v.lon_control_mode, ros_v.lon_control_mode, type);
	convert(old_ros_v.throttle, ros_v.throttle, type);
	convert(old_ros_v.brake, ros_v.brake, type);
	convert(old_ros_v.speed, ros_v.speed, type);
	convert(old_ros_v.acceleration, ros_v.acceleration, type);
	convert(old_ros_v.axle_torque, ros_v.axle_torque, type);
	convert(old_ros_v.stop_distance, ros_v.stop_distance, type);
	convert(old_ros_v.gear_command_value, ros_v.gear_command_value, type);
	convert(old_ros_v.turn_signal_cmd, ros_v.turn_signal_cmd, type);
	convert(old_ros_v.light_signal_cmd, ros_v.light_signal_cmd, type);
	convert(old_ros_v.horn_signal_cmd, ros_v.horn_signal_cmd, type);
	convert(old_ros_v.rear_view_mirror_signal_cmd, ros_v.rear_view_mirror_signal_cmd, type);
	convert(old_ros_v.control_trajectory, ros_v.control_trajectory, type);
	convert(old_ros_v.control_status, ros_v.control_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ControlStatus &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.control_status_type, ros_v.control_status_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ControlTrajectory &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.control_result_points_size, ros_v.control_result_points_size, type);
	ros_v.control_result_points.resize(old_ros_v.control_result_points.size());
	for (int i = 0; i < ros_v.control_result_points.size(); i++) {
	    convert(old_ros_v.control_result_points[i], ros_v.control_result_points[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LatControlMode &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.lat_control_mode, ros_v.lat_control_mode, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LonControlMode &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.lon_control_mode, ros_v.lon_control_mode, type);
}

REG_CONVERT_SINGLE(_iflytek_control_control_command_converter, "/iflytek/control/control_command", ControlOutput);
