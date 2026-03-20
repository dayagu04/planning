#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AD2HMIData.h"
#include "struct_msgs_v2_10/AD2HMIData.h"
#include "struct_msgs/ALCOutputInfoStr.h"
#include "struct_msgs_v2_10/ALCOutputInfoStr.h"
#include "struct_msgs/APAHMIData.h"
#include "struct_msgs_v2_10/APAHMIData.h"
#include "struct_msgs/CIPVInfoStr.h"
#include "struct_msgs_v2_10/CIPVInfoStr.h"
#include "struct_msgs/ELKOutputInfoStr.h"
#include "struct_msgs_v2_10/ELKOutputInfoStr.h"
#include "struct_msgs/HPPHMIData.h"
#include "struct_msgs_v2_10/HPPHMIData.h"
#include "struct_msgs/IHCOutputInfoStr.h"
#include "struct_msgs_v2_10/IHCOutputInfoStr.h"
#include "struct_msgs/LCCSTrajectoryPoint.h"
#include "struct_msgs_v2_10/LCCSTrajectoryPoint.h"
#include "struct_msgs/LCCS_TrajPoly.h"
#include "struct_msgs_v2_10/LCCS_TrajPoly.h"
#include "struct_msgs/LDPOutputInfoStr.h"
#include "struct_msgs_v2_10/LDPOutputInfoStr.h"
#include "struct_msgs/LDWOutputInfoStr.h"
#include "struct_msgs_v2_10/LDWOutputInfoStr.h"
#include "struct_msgs/LandingPoint.h"
#include "struct_msgs_v2_10/LandingPoint.h"
#include "struct_msgs/ObstacleInfo.h"
#include "struct_msgs_v2_10/ObstacleInfo.h"
#include "struct_msgs/PlanningHMIOutputInfoStr.h"
#include "struct_msgs_v2_10/PlanningHMIOutputInfoStr.h"
#include "struct_msgs/TLAOutputInfoStr.h"
#include "struct_msgs_v2_10/TLAOutputInfoStr.h"
#include "struct_msgs/TSROutputInfoStr.h"
#include "struct_msgs_v2_10/TSROutputInfoStr.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AD2HMIData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.is_avaliable, ros_v.is_avaliable, type);
	convert(old_ros_v.timestamp, ros_v.timestamp, type);
	convert(old_ros_v.lane_change_direction, ros_v.lane_change_direction, type);
	convert(old_ros_v.lane_change_status, ros_v.lane_change_status, type);
	convert(old_ros_v.lane_change_reason, ros_v.lane_change_reason, type);
	convert(old_ros_v.status_update_reason, ros_v.status_update_reason, type);
	convert(old_ros_v.noa_exit_warning_level_distance, ros_v.noa_exit_warning_level_distance, type);
	convert(old_ros_v.avoid_status, ros_v.avoid_status, type);
	convert(old_ros_v.aovid_id, ros_v.aovid_id, type);
	convert(old_ros_v.is_curva, ros_v.is_curva, type);
	convert(old_ros_v.landing_point, ros_v.landing_point, type);
	convert(old_ros_v.tp, ros_v.tp, type);
	convert(old_ros_v.obstacle_info_size, ros_v.obstacle_info_size, type);
	ros_v.obstacle_info.resize(old_ros_v.obstacle_info.size());
	for (int i = 0; i < ros_v.obstacle_info.size(); i++) {
	    convert(old_ros_v.obstacle_info[i], ros_v.obstacle_info[i], type);
	}
	convert(old_ros_v.cutin_track_id, ros_v.cutin_track_id, type);
	convert(old_ros_v.cruise_speed, ros_v.cruise_speed, type);
	convert(old_ros_v.avoiddirect, ros_v.avoiddirect, type);
	convert(old_ros_v.distance_to_ramp, ros_v.distance_to_ramp, type);
	convert(old_ros_v.distance_to_split, ros_v.distance_to_split, type);
	convert(old_ros_v.distance_to_merge, ros_v.distance_to_merge, type);
	convert(old_ros_v.distance_to_toll_station, ros_v.distance_to_toll_station, type);
	convert(old_ros_v.distance_to_tunnel, ros_v.distance_to_tunnel, type);
	convert(old_ros_v.is_within_hdmap, ros_v.is_within_hdmap, type);
	convert(old_ros_v.ramp_direction, ros_v.ramp_direction, type);
	convert(old_ros_v.ramp_pass_sts, ros_v.ramp_pass_sts, type);
	convert(old_ros_v.dis_to_reference_line, ros_v.dis_to_reference_line, type);
	convert(old_ros_v.angle_to_roaddirection, ros_v.angle_to_roaddirection, type);
	convert(old_ros_v.is_in_sdmaproad, ros_v.is_in_sdmaproad, type);
	convert(old_ros_v.road_type, ros_v.road_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ALCOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lc_request_size, ros_v.lc_request_size, type);
	ros_v.lc_request.resize(old_ros_v.lc_request.size());
	for (int i = 0; i < ros_v.lc_request.size(); i++) {
	    convert(old_ros_v.lc_request[i], ros_v.lc_request[i], type);
	}
	convert(old_ros_v.lc_status_size, ros_v.lc_status_size, type);
	ros_v.lc_status.resize(old_ros_v.lc_status.size());
	for (int i = 0; i < ros_v.lc_status.size(); i++) {
	    convert(old_ros_v.lc_status[i], ros_v.lc_status[i], type);
	}
	convert(old_ros_v.lc_invalid_reason_size, ros_v.lc_invalid_reason_size, type);
	ros_v.lc_invalid_reason.resize(old_ros_v.lc_invalid_reason.size());
	for (int i = 0; i < ros_v.lc_invalid_reason.size(); i++) {
	    convert(old_ros_v.lc_invalid_reason[i], ros_v.lc_invalid_reason[i], type);
	}
	convert(old_ros_v.lc_back_reason_size, ros_v.lc_back_reason_size, type);
	ros_v.lc_back_reason.resize(old_ros_v.lc_back_reason.size());
	for (int i = 0; i < ros_v.lc_back_reason.size(); i++) {
	    convert(old_ros_v.lc_back_reason[i], ros_v.lc_back_reason[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::APAHMIData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.remain_dist, ros_v.remain_dist, type);
	convert(old_ros_v.is_parking_pause, ros_v.is_parking_pause, type);
	convert(old_ros_v.parking_pause_reason, ros_v.parking_pause_reason, type);
	convert(old_ros_v.prepare_plan_state, ros_v.prepare_plan_state, type);
	convert(old_ros_v.planning_park_dir, ros_v.planning_park_dir, type);
	convert(old_ros_v.pa_remain_distance, ros_v.pa_remain_distance, type);
	convert(old_ros_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
	convert(old_ros_v.planning_park_pa_dir, ros_v.planning_park_pa_dir, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CIPVInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.has_cipv, ros_v.has_cipv, type);
	convert(old_ros_v.cipv_id, ros_v.cipv_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ELKOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.elk_state, ros_v.elk_state, type);
	convert(old_ros_v.elk_left_intervention_flag, ros_v.elk_left_intervention_flag, type);
	convert(old_ros_v.elk_right_intervention_flag, ros_v.elk_right_intervention_flag, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HPPHMIData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.is_avaliable, ros_v.is_avaliable, type);
	convert(old_ros_v.distance_to_parking_space, ros_v.distance_to_parking_space, type);
	convert(old_ros_v.avoid_status, ros_v.avoid_status, type);
	convert(old_ros_v.avoid_obstacle_type, ros_v.avoid_obstacle_type, type);
	convert(old_ros_v.aovid_id, ros_v.aovid_id, type);
	convert(old_ros_v.is_approaching_turn, ros_v.is_approaching_turn, type);
	convert(old_ros_v.is_left_turn, ros_v.is_left_turn, type);
	convert(old_ros_v.is_approaching_intersection, ros_v.is_approaching_intersection, type);
	convert(old_ros_v.is_approaching_speed_bumps, ros_v.is_approaching_speed_bumps, type);
	convert(old_ros_v.emergency_level, ros_v.emergency_level, type);
	convert(old_ros_v.is_parking_space_occupied, ros_v.is_parking_space_occupied, type);
	convert(old_ros_v.is_new_parking_space_found, ros_v.is_new_parking_space_found, type);
	convert(old_ros_v.is_on_hpp_lane, ros_v.is_on_hpp_lane, type);
	convert(old_ros_v.is_reached_hpp_trace_start, ros_v.is_reached_hpp_trace_start, type);
	convert(old_ros_v.accumulated_driving_distance, ros_v.accumulated_driving_distance, type);
	convert(old_ros_v.estimated_remaining_time, ros_v.estimated_remaining_time, type);
	convert(old_ros_v.hpp_state_switch, ros_v.hpp_state_switch, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RADSHMIData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.is_avaliable, ros_v.is_avaliable, type);
	convert(old_ros_v.rads_pause_reason, ros_v.rads_pause_reason, type);
	convert(old_ros_v.avoid_status, ros_v.avoid_status, type);
	convert(old_ros_v.aovid_id, ros_v.aovid_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IHCOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ihc_state, ros_v.ihc_state, type);
	convert(old_ros_v.ihc_request_status, ros_v.ihc_request_status, type);
	convert(old_ros_v.ihc_request, ros_v.ihc_request, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LCCSTrajectoryPoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.start, ros_v.start, type);
	convert(old_ros_v.middle, ros_v.middle, type);
	convert(old_ros_v.end, ros_v.end, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LCCS_TrajPoly &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.c0, ros_v.c0, type);
	convert(old_ros_v.c1, ros_v.c1, type);
	convert(old_ros_v.c2, ros_v.c2, type);
	convert(old_ros_v.c3, ros_v.c3, type);
	convert(old_ros_v.c4, ros_v.c4, type);
	convert(old_ros_v.c5, ros_v.c5, type);
	convert(old_ros_v.start_x, ros_v.start_x, type);
	convert(old_ros_v.end_x, ros_v.end_x, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LDPOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ldp_state, ros_v.ldp_state, type);
	convert(old_ros_v.ldp_left_intervention_flag, ros_v.ldp_left_intervention_flag, type);
	convert(old_ros_v.ldp_right_intervention_flag, ros_v.ldp_right_intervention_flag, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LDWOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ldw_state, ros_v.ldw_state, type);
	convert(old_ros_v.ldw_left_warning, ros_v.ldw_left_warning, type);
	convert(old_ros_v.ldw_right_warning, ros_v.ldw_right_warning, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LandingPoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.relative_pos, ros_v.relative_pos, type);
	convert(old_ros_v.heading, ros_v.heading, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ObstacleInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.speed_x, ros_v.speed_x, type);
	convert(old_ros_v.speed_y, ros_v.speed_y, type);
	convert(old_ros_v.heading, ros_v.heading, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.center_x, ros_v.center_x, type);
	convert(old_ros_v.center_y, ros_v.center_y, type);
	convert(old_ros_v.size, ros_v.size, type);
	convert(old_ros_v.lon_status, ros_v.lon_status, type);
	convert(old_ros_v.anchor, ros_v.anchor, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PlanningHMIOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.ldw_output_info, ros_v.ldw_output_info, type);
	convert(old_ros_v.ldp_output_info, ros_v.ldp_output_info, type);
	convert(old_ros_v.elk_output_info, ros_v.elk_output_info, type);
	convert(old_ros_v.tsr_output_info, ros_v.tsr_output_info, type);
	convert(old_ros_v.ihc_output_info, ros_v.ihc_output_info, type);
	convert(old_ros_v.alc_output_info, ros_v.alc_output_info, type);
	convert(old_ros_v.cipv_info, ros_v.cipv_info, type);
	convert(old_ros_v.tla_output_info, ros_v.tla_output_info, type);
	convert(old_ros_v.ad_info, ros_v.ad_info, type);
	convert(old_ros_v.hpp_info, ros_v.hpp_info, type);
	convert(old_ros_v.apa_info, ros_v.apa_info, type);
	convert(old_ros_v.nsa_info, ros_v.nsa_info, type);
	convert(old_ros_v.rads_info, ros_v.rads_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TLAOutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.traffic_light_reminder, ros_v.traffic_light_reminder, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TSROutputInfoStr &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.tsr_state, ros_v.tsr_state, type);
	convert(old_ros_v.tsr_speed_limit, ros_v.tsr_speed_limit, type);
	convert(old_ros_v.tsr_warning, ros_v.tsr_warning, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::NSAHMIData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.is_avaliable, ros_v.is_avaliable, type);
	convert(old_ros_v.nsa_disable_reason, ros_v.nsa_disable_reason, type);
	convert(old_ros_v.is_complete, ros_v.is_complete, type);
	convert(old_ros_v.nsa_complete_reason, ros_v.nsa_complete_reason, type);
	convert(old_ros_v.nsa_pause_reason, ros_v.nsa_pause_reason, type);
}

REG_CONVERT_SINGLE(_iflytek_planning_hmi_converter, "/iflytek/planning/hmi", PlanningHMIOutputInfoStr);
