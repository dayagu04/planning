#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AccelerationRange.h"
#include "struct_msgs_v2_10/AccelerationRange.h"
#include "struct_msgs/GearCommand.h"
#include "struct_msgs_v2_10/GearCommand.h"
#include "struct_msgs/HornSignalCommand.h"
#include "struct_msgs_v2_10/HornSignalCommand.h"
#include "struct_msgs/LightSignalCommand.h"
#include "struct_msgs_v2_10/LightSignalCommand.h"
#include "struct_msgs/OpenLoopSteeringCommand.h"
#include "struct_msgs_v2_10/OpenLoopSteeringCommand.h"
#include "struct_msgs/PlanMeta.h"
#include "struct_msgs_v2_10/PlanMeta.h"
#include "struct_msgs/PlanningOutput.h"
#include "struct_msgs_v2_10/PlanningOutput.h"
#include "struct_msgs/PlanningRequest.h"
#include "struct_msgs_v2_10/PlanningRequest.h"
#include "struct_msgs/PlanningStatus.h"
#include "struct_msgs_v2_10/PlanningStatus.h"
#include "struct_msgs/RearViewMirrorCommand.h"
#include "struct_msgs_v2_10/RearViewMirrorCommand.h"
#include "struct_msgs/SuccessfulSlotsInfo.h"
#include "struct_msgs_v2_10/SuccessfulSlotsInfo.h"
#include "struct_msgs/TargetReference.h"
#include "struct_msgs_v2_10/TargetReference.h"
#include "struct_msgs/Trajectory.h"
#include "struct_msgs_v2_10/Trajectory.h"
#include "struct_msgs/TrajectoryPoint.h"
#include "struct_msgs_v2_10/TrajectoryPoint.h"
#include "struct_msgs/TurnSignalCommand.h"
#include "struct_msgs_v2_10/TurnSignalCommand.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AccelerationRange &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.min_a, ros_v.min_a, type);
	convert(old_ros_v.max_a, ros_v.max_a, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::GearCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.gear_command_value, ros_v.gear_command_value, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HornSignalCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.horn_signal_value, ros_v.horn_signal_value, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LightSignalCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.light_signal_value, ros_v.light_signal_value, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::OpenLoopSteeringCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.jerk_factor, ros_v.jerk_factor, type);
	convert(old_ros_v.need_steering_wheel_stationary, ros_v.need_steering_wheel_stationary, type);
	convert(old_ros_v.steering_wheel_rad_limit, ros_v.steering_wheel_rad_limit, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PlanMeta &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.plan_strategy_name[i], ros_v.plan_strategy_name[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PlanningOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.meta, ros_v.meta, type);
	convert(old_ros_v.trajectory, ros_v.trajectory, type);
	convert(old_ros_v.turn_signal_command, ros_v.turn_signal_command, type);
	convert(old_ros_v.light_signal_command, ros_v.light_signal_command, type);
	convert(old_ros_v.horn_signal_command, ros_v.horn_signal_command, type);
	convert(old_ros_v.gear_command, ros_v.gear_command, type);
	convert(old_ros_v.rear_view_mirror_signal_command, ros_v.rear_view_mirror_signal_command, type);
	convert(old_ros_v.open_loop_steering_command, ros_v.open_loop_steering_command, type);
	convert(old_ros_v.planning_status, ros_v.planning_status, type);
	convert(old_ros_v.successful_slot_info_list_size, ros_v.successful_slot_info_list_size, type);
	ros_v.successful_slot_info_list.resize(old_ros_v.successful_slot_info_list.size());
	for (int i = 0; i < ros_v.successful_slot_info_list.size(); i++) {
	    convert(old_ros_v.successful_slot_info_list[i], ros_v.successful_slot_info_list[i], type);
	}
	convert(old_ros_v.planning_request, ros_v.planning_request, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PlanningRequest &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.take_over_req_level, ros_v.take_over_req_level, type);
	convert(old_ros_v.request_reason, ros_v.request_reason, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PlanningStatus &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.standstill, ros_v.standstill, type);
	convert(old_ros_v.ready_to_go, ros_v.ready_to_go, type);
	convert(old_ros_v.apa_planning_status, ros_v.apa_planning_status, type);
	convert(old_ros_v.hpp_planning_status, ros_v.hpp_planning_status, type);
	convert(old_ros_v.rads_planning_status, ros_v.rads_planning_status, type);
	convert(old_ros_v.nsa_planning_status, ros_v.nsa_planning_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RearViewMirrorCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.rear_view_mirror_value, ros_v.rear_view_mirror_value, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SuccessfulSlotsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TargetReference &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.polynomial[i], ros_v.polynomial[i], type);
	}
	convert(old_ros_v.target_velocity, ros_v.target_velocity, type);
	convert(old_ros_v.acceleration_range_limit, ros_v.acceleration_range_limit, type);
	convert(old_ros_v.lateral_maneuver_gear, ros_v.lateral_maneuver_gear, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Trajectory &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.trajectory_type, ros_v.trajectory_type, type);
	convert(old_ros_v.target_reference, ros_v.target_reference, type);
	convert(old_ros_v.trajectory_points_size, ros_v.trajectory_points_size, type);
	ros_v.trajectory_points.resize(old_ros_v.trajectory_points.size());
	for (int i = 0; i < ros_v.trajectory_points.size(); i++) {
	    convert(old_ros_v.trajectory_points[i], ros_v.trajectory_points[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TrajectoryPoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.heading_yaw, ros_v.heading_yaw, type);
	convert(old_ros_v.curvature, ros_v.curvature, type);
	convert(old_ros_v.t, ros_v.t, type);
	convert(old_ros_v.v, ros_v.v, type);
	convert(old_ros_v.a, ros_v.a, type);
	convert(old_ros_v.distance, ros_v.distance, type);
	convert(old_ros_v.jerk, ros_v.jerk, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TurnSignalCommand &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.turn_signal_value, ros_v.turn_signal_value, type);
}

REG_CONVERT_SINGLE(_iflytek_planning_plan_converter, "/iflytek/planning/plan", PlanningOutput);
