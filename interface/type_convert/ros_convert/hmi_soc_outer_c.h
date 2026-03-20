#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/HmiAccInfo.h"
#include "struct_msgs_v2_10/HmiAccInfo.h"
#include "struct_msgs/HmiAdasInfo.h"
#include "struct_msgs_v2_10/HmiAdasInfo.h"
#include "struct_msgs/HmiApaInfo.h"
#include "struct_msgs_v2_10/HmiApaInfo.h"
#include "struct_msgs/HmiApaSlotInfo.h"
#include "struct_msgs_v2_10/HmiApaSlotInfo.h"
#include "struct_msgs/HmiCommon.h"
#include "struct_msgs_v2_10/HmiCommon.h"
#include "struct_msgs/HmiHppOutput.h"
#include "struct_msgs_v2_10/HmiHppOutput.h"
#include "struct_msgs/HmiIntelligentEvasion.h"
#include "struct_msgs_v2_10/HmiIntelligentEvasion.h"
#include "struct_msgs/HmiLaneChange.h"
#include "struct_msgs_v2_10/HmiLaneChange.h"
#include "struct_msgs/HmiLaneGroundMarking.h"
#include "struct_msgs_v2_10/HmiLaneGroundMarking.h"
#include "struct_msgs/HmiLaneInfo.h"
#include "struct_msgs_v2_10/HmiLaneInfo.h"
#include "struct_msgs/HmiLineInfo.h"
#include "struct_msgs_v2_10/HmiLineInfo.h"
#include "struct_msgs/HmiNoaInfo.h"
#include "struct_msgs_v2_10/HmiNoaInfo.h"
#include "struct_msgs/HmiObjInfo.h"
#include "struct_msgs_v2_10/HmiObjInfo.h"
#include "struct_msgs/HmiRadsInfo.h"
#include "struct_msgs_v2_10/HmiRadsInfo.h"
#include "struct_msgs/HmiRpaInfo.h"
#include "struct_msgs_v2_10/HmiRpaInfo.h"
#include "struct_msgs/HmiSccInfo.h"
#include "struct_msgs_v2_10/HmiSccInfo.h"
#include "struct_msgs/HmiSensorInfo.h"
#include "struct_msgs_v2_10/HmiSensorInfo.h"
#include "struct_msgs/HmiSocOuter.h"
#include "struct_msgs_v2_10/HmiSocOuter.h"
#include "struct_msgs/HmiStopLine.h"
#include "struct_msgs_v2_10/HmiStopLine.h"
#include "struct_msgs/HmiSwitchInfo.h"
#include "struct_msgs_v2_10/HmiSwitchInfo.h"
#include "struct_msgs/Traj_Point.h"
#include "struct_msgs_v2_10/Traj_Point.h"
#include "struct_msgs/TrajectoryPointSet.h"
#include "struct_msgs_v2_10/TrajectoryPointSet.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiAccInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.acc_status, ros_v.acc_status, type);
	convert(old_ros_v.acc_active_resp, ros_v.acc_active_resp, type);
	convert(old_ros_v.acc_driver_denied, ros_v.acc_driver_denied, type);
	convert(old_ros_v.acc_takeover_req_lv, ros_v.acc_takeover_req_lv, type);
	convert(old_ros_v.acc_set_headway, ros_v.acc_set_headway, type);
	convert(old_ros_v.acc_set_speed, ros_v.acc_set_speed, type);
	convert(old_ros_v.intelligent_following, ros_v.intelligent_following, type);
	convert(old_ros_v.acc_notify_req, ros_v.acc_notify_req, type);
	// warning : added struct member : int16 acc_notify
	// warning : added struct member : int16 distraction_warning
	// warning : added struct member : SpeedOffsetInfo speed_offset_set
	// warning : added struct member : PilotUserPreference pilot_user_preference
	// warning : added struct member : int16 acc_spd_mode_ind
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiAdasInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ldw_output_info, ros_v.ldw_output_info, type);
	convert(old_ros_v.ldp_output_info, ros_v.ldp_output_info, type);
	convert(old_ros_v.elk_output_info, ros_v.elk_output_info, type);
	convert(old_ros_v.tsr_output_info, ros_v.tsr_output_info, type);
	convert(old_ros_v.ihc_output_info, ros_v.ihc_output_info, type);
	convert(old_ros_v.supp_signs_size, ros_v.supp_signs_size, type);
	ros_v.supp_signs.resize(old_ros_v.supp_signs.size());
	for (int i = 0; i < ros_v.supp_signs.size(); i++) {
	    convert(old_ros_v.supp_signs[i], ros_v.supp_signs[i], type);
	}
	convert(old_ros_v.traffic_lights_size, ros_v.traffic_lights_size, type);
	ros_v.traffic_lights.resize(old_ros_v.traffic_lights.size());
	for (int i = 0; i < ros_v.traffic_lights.size(); i++) {
	    convert(old_ros_v.traffic_lights[i], ros_v.traffic_lights[i], type);
	}
	// warning : added struct member : AMAPOutputInfoStr ama_output_info
	// warning : added struct member : MEBOutputInfoStr meb_output_info
	// warning : added struct member : CameraPerceptionTrafficLight current_lane_traffic_light
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiApaInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.apa_mode_status, ros_v.apa_mode_status, type);
	convert(old_ros_v.apa_status, ros_v.apa_status, type);
	convert(old_ros_v.apa_slot_info_size, ros_v.apa_slot_info_size, type);
	ros_v.apa_slot_info.resize(old_ros_v.apa_slot_info.size());
	for (int i = 0; i < ros_v.apa_slot_info.size(); i++) {
	    convert(old_ros_v.apa_slot_info[i], ros_v.apa_slot_info[i], type);
	}
	convert(old_ros_v.select_slot_id, ros_v.select_slot_id, type);
	convert(old_ros_v.apa_notify_req, ros_v.apa_notify_req, type);
	convert(old_ros_v.apa_notify_remain_distance, ros_v.apa_notify_remain_distance, type);
	convert(old_ros_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
	convert(old_ros_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
	convert(old_ros_v.prepare_plan_state, ros_v.prepare_plan_state, type);
	convert(old_ros_v.planning_park_dir, ros_v.planning_park_dir, type);
	convert(old_ros_v.select_park_out_dir, ros_v.select_park_out_dir, type);
	convert(old_ros_v.select_park_in_dir, ros_v.select_park_in_dir, type);
	convert(old_ros_v.parking_time, ros_v.parking_time, type);
	convert(old_ros_v.hmi_rap_info, ros_v.hmi_rap_info, type);
	convert(old_ros_v.apa_active_denied, ros_v.apa_active_denied, type);
	// warning : modified struct member : int16 apa_active_resp -> bool apa_active_resp
	convert(old_ros_v.apa_active_resp, ros_v.apa_active_resp, type);
	// warning : added struct member : bool blue_light_state
	// warning : added struct member : int16 avm_screen_view
	// warning : added struct member : int16 apa_active_button_sts
	// warning : added struct member : int16 park_in_button_sts
	// warning : added struct member : int16 park_out_button_sts
	// warning : added struct member : int16 free_park_button_sts
	// warning : added struct member : int16 apa_start_button_sts
	// warning : added struct member : int16 apa_resume_button_sts
	// warning : added struct member : int16 rpa_active_button_sts
	// warning : added struct member : int16 leave_car_park_button_sts
	// warning : added struct member : ApaUserPreference apa_user_preference
	// warning : added struct member : int16 parking_speed_set
	// warning : added struct member : int16 parking_view_ctrl
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiApaSlotInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.apa_slot_id, ros_v.apa_slot_id, type);
	convert(old_ros_v.apa_slot_type, ros_v.apa_slot_type, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.apa_slot_corner_points[i], ros_v.apa_slot_corner_points[i], type);
	}
	convert(old_ros_v.allow_parking, ros_v.allow_parking, type);
	convert(old_ros_v.planning_success, ros_v.planning_success, type);
	convert(old_ros_v.apa_slot_num, ros_v.apa_slot_num, type);
	convert(old_ros_v.limiters_size, ros_v.limiters_size, type);
	ros_v.limiters.resize(old_ros_v.limiters.size());
	for (int i = 0; i < ros_v.limiters.size(); i++) {
	    convert(old_ros_v.limiters[i], ros_v.limiters[i], type);
	}
	// warning : added struct member : bool is_narrow_slot
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiCommon &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.common_notify_req, ros_v.common_notify_req, type);
	convert(old_ros_v.vehicle_speed_display, ros_v.vehicle_speed_display, type);
	convert(old_ros_v.brake_pedal_pressed, ros_v.brake_pedal_pressed, type);
	convert(old_ros_v.shift_lever_state, ros_v.shift_lever_state, type);
	convert(old_ros_v.turn_light_state, ros_v.turn_light_state, type);
	convert(old_ros_v.tp_set, ros_v.tp_set, type);
	convert(old_ros_v.position, ros_v.position, type);
	// warning : added struct member : RearViewMirrorCommand rear_view_mirror_signal_command
	// warning : added struct member : bool adas_takeover_req
	// warning : added struct member : int16 adas_takeover_reason
	// warning : added struct member : int16 CruiseAccelerateSts
	// warning : added struct member : int16 avoiddirect
	// warning : added struct member : int16 borrow_direction
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiHppOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.hpp_notify_req, ros_v.hpp_notify_req, type);
	convert(old_ros_v.hpp_status, ros_v.hpp_status, type);
	convert(old_ros_v.memory_parking_available, ros_v.memory_parking_available, type);
	convert(old_ros_v.is_first_time_using, ros_v.is_first_time_using, type);
	convert(old_ros_v.memory_parking_resume_available, ros_v.memory_parking_resume_available, type);
	convert(old_ros_v.route_learning_available, ros_v.route_learning_available, type);
	convert(old_ros_v.speed_bumps_count, ros_v.speed_bumps_count, type);
	convert(old_ros_v.learning_distance, ros_v.learning_distance, type);
	convert(old_ros_v.distance_to_parking_space, ros_v.distance_to_parking_space, type);
	convert(old_ros_v.estimated_remaining_time, ros_v.estimated_remaining_time, type);
	convert(old_ros_v.pedestrian_avoidance_count, ros_v.pedestrian_avoidance_count, type);
	convert(old_ros_v.vehicle_avoidance_count, ros_v.vehicle_avoidance_count, type);
	convert(old_ros_v.hpp_time_minute, ros_v.hpp_time_minute, type);
	convert(old_ros_v.hpp_takeover_req_lv, ros_v.hpp_takeover_req_lv, type);
	convert(old_ros_v.is_position_need_refresh, ros_v.is_position_need_refresh, type);
	convert(old_ros_v.his_traj_point_size, ros_v.his_traj_point_size, type);
	ros_v.his_traj_point.resize(old_ros_v.his_traj_point.size());
	for (int i = 0; i < ros_v.his_traj_point.size(); i++) {
	    convert(old_ros_v.his_traj_point[i], ros_v.his_traj_point[i], type);
	}
	convert(old_ros_v.start_point, ros_v.start_point, type);
	convert(old_ros_v.end_point, ros_v.end_point, type);
	convert(old_ros_v.hpp_active_resp, ros_v.hpp_active_resp, type);
	convert(old_ros_v.hpp_active_denied, ros_v.hpp_active_denied, type);
	// warning : added struct member : int16 hpa_active_button_sts
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiIntelligentEvasion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.dodge_type, ros_v.dodge_type, type);
	convert(old_ros_v.object_id, ros_v.object_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiLaneChange &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lc_status, ros_v.lc_status, type);
	convert(old_ros_v.lc_direction, ros_v.lc_direction, type);
	convert(old_ros_v.lc_reason, ros_v.lc_reason, type);
	convert(old_ros_v.obstacle_id, ros_v.obstacle_id, type);
	convert(old_ros_v.landing_point, ros_v.landing_point, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiLaneGroundMarking &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.ground_marking_points_set[i], ros_v.ground_marking_points_set[i], type);
	}
	convert(old_ros_v.orientation_angle, ros_v.orientation_angle, type);
	convert(old_ros_v.turn_type, ros_v.turn_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiLaneInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.relative_id, ros_v.relative_id, type);
	convert(old_ros_v.lane_types_size, ros_v.lane_types_size, type);
	ros_v.lane_types.resize(old_ros_v.lane_types.size());
	for (int i = 0; i < ros_v.lane_types.size(); i++) {
	    convert(old_ros_v.lane_types[i], ros_v.lane_types[i], type);
	}
	convert(old_ros_v.merge_split_points_size, ros_v.merge_split_points_size, type);
	ros_v.merge_split_points.resize(old_ros_v.merge_split_points.size());
	for (int i = 0; i < ros_v.merge_split_points.size(); i++) {
	    convert(old_ros_v.merge_split_points[i], ros_v.merge_split_points[i], type);
	}
	convert(old_ros_v.speed_limit, ros_v.speed_limit, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiLineInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.line_index, ros_v.line_index, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.line_segments_size, ros_v.line_segments_size, type);
	ros_v.line_segments.resize(old_ros_v.line_segments.size());
	for (int i = 0; i < ros_v.line_segments.size(); i++) {
	    convert(old_ros_v.line_segments[i], ros_v.line_segments[i], type);
	}
	convert(old_ros_v.marking_segments_size, ros_v.marking_segments_size, type);
	ros_v.marking_segments.resize(old_ros_v.marking_segments.size());
	for (int i = 0; i < ros_v.marking_segments.size(); i++) {
	    convert(old_ros_v.marking_segments[i], ros_v.marking_segments[i], type);
	}
	convert(old_ros_v.color_segments_size, ros_v.color_segments_size, type);
	ros_v.color_segments.resize(old_ros_v.color_segments.size());
	for (int i = 0; i < ros_v.color_segments.size(); i++) {
	    convert(old_ros_v.color_segments[i], ros_v.color_segments[i], type);
	}
	// warning : added struct member : uint32 line_id
	// warning : added struct member : int8 car_points_size
	// warning : added struct member : Point2f[] car_points
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiNoaInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.noa_status, ros_v.noa_status, type);
	convert(old_ros_v.noa_activate_resp, ros_v.noa_activate_resp, type);
	convert(old_ros_v.noa_driver_denied, ros_v.noa_driver_denied, type);
	convert(old_ros_v.noa_takeover_req_lv, ros_v.noa_takeover_req_lv, type);
	convert(old_ros_v.distance_to_destination, ros_v.distance_to_destination, type);
	convert(old_ros_v.distance_to_ramp, ros_v.distance_to_ramp, type);
	convert(old_ros_v.distance_to_tunnel, ros_v.distance_to_tunnel, type);
	convert(old_ros_v.distance_to_split, ros_v.distance_to_split, type);
	convert(old_ros_v.distance_to_merge, ros_v.distance_to_merge, type);
	convert(old_ros_v.noa_notify_req, ros_v.noa_notify_req, type);
	// warning : added struct member : uint32 distance_to_station
	// warning : added struct member : int16 noa_notify
	// warning : added struct member : int16 lane_change_style
	// warning : added struct member : bool noa_cruise_dclc_resp
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiObjInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.target_long_position, ros_v.target_long_position, type);
	convert(old_ros_v.target_lat_position, ros_v.target_lat_position, type);
	convert(old_ros_v.heading_angle, ros_v.heading_angle, type);
	convert(old_ros_v.target_track_id, ros_v.target_track_id, type);
	convert(old_ros_v.target_type, ros_v.target_type, type);
	convert(old_ros_v.motion_pattern, ros_v.motion_pattern, type);
	convert(old_ros_v.light_status, ros_v.light_status, type);
	convert(old_ros_v.shape, ros_v.shape, type);
	convert(old_ros_v.velocity, ros_v.velocity, type);
	convert(old_ros_v.lane_id, ros_v.lane_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiRadsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rads_status, ros_v.rads_status, type);
	convert(old_ros_v.rads_notify_req, ros_v.rads_notify_req, type);
	convert(old_ros_v.rads_remain_distance, ros_v.rads_remain_distance, type);
	convert(old_ros_v.rads_active_resp, ros_v.rads_active_resp, type);
	convert(old_ros_v.rads_active_denied, ros_v.rads_active_denied, type);
	// warning : added struct member : float32 remain_distance_percentage
	// warning : added struct member : uint16 path_point_size
	// warning : added struct member : Point2f[] path_point
	// warning : added struct member : int16 rads_active_button_sts
	// warning : added struct member : int16 rads_start_button_sts
	// warning : added struct member : int16 rads_continue_button_sts
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiRpaInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.rpa_available, ros_v.rpa_available, type);
	convert(old_ros_v.rpa_active_resp, ros_v.rpa_active_resp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSccInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.scc_status, ros_v.scc_status, type);
	convert(old_ros_v.scc_active_resp, ros_v.scc_active_resp, type);
	convert(old_ros_v.scc_driver_denied, ros_v.scc_driver_denied, type);
	convert(old_ros_v.scc_hands_off_warning, ros_v.scc_hands_off_warning, type);
	convert(old_ros_v.scc_takeover_req_lv, ros_v.scc_takeover_req_lv, type);
	convert(old_ros_v.scc_line_detect_status, ros_v.scc_line_detect_status, type);
	convert(old_ros_v.intelligent_evasion, ros_v.intelligent_evasion, type);
	convert(old_ros_v.lane_change, ros_v.lane_change, type);
	convert(old_ros_v.narrow_road_tips, ros_v.narrow_road_tips, type);
	convert(old_ros_v.scc_notify_req, ros_v.scc_notify_req, type);
	// warning : added struct member : int16 scc_notify
	// warning : added struct member : int16 hands_off_detection
	// warning : added struct member : bool traffic_light_stop_go
	// warning : added struct member : bool obstacle_bypass
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSensorInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.sensor_type, ros_v.sensor_type, type);
	convert(old_ros_v.sensor_state, ros_v.sensor_state, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSocOuter &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.hmi_common, ros_v.hmi_common, type);
	convert(old_ros_v.hmi_line_topo_size, ros_v.hmi_line_topo_size, type);
	ros_v.hmi_line_topo.resize(old_ros_v.hmi_line_topo.size());
	for (int i = 0; i < ros_v.hmi_line_topo.size(); i++) {
	    convert(old_ros_v.hmi_line_topo[i], ros_v.hmi_line_topo[i], type);
	}
	convert(old_ros_v.hmi_line_size, ros_v.hmi_line_size, type);
	ros_v.hmi_line.resize(old_ros_v.hmi_line.size());
	for (int i = 0; i < ros_v.hmi_line.size(); i++) {
	    convert(old_ros_v.hmi_line[i], ros_v.hmi_line[i], type);
	}
	convert(old_ros_v.hmi_stop_line_size, ros_v.hmi_stop_line_size, type);
	ros_v.hmi_stop_line.resize(old_ros_v.hmi_stop_line.size());
	for (int i = 0; i < ros_v.hmi_stop_line.size(); i++) {
	    convert(old_ros_v.hmi_stop_line[i], ros_v.hmi_stop_line[i], type);
	}
	convert(old_ros_v.hmi_inhibit_line_size, ros_v.hmi_inhibit_line_size, type);
	ros_v.hmi_inhibit_line.resize(old_ros_v.hmi_inhibit_line.size());
	for (int i = 0; i < ros_v.hmi_inhibit_line.size(); i++) {
	    convert(old_ros_v.hmi_inhibit_line[i], ros_v.hmi_inhibit_line[i], type);
	}
	convert(old_ros_v.hmi_lane_ground_marking_size, ros_v.hmi_lane_ground_marking_size, type);
	ros_v.hmi_lane_ground_marking.resize(old_ros_v.hmi_lane_ground_marking.size());
	for (int i = 0; i < ros_v.hmi_lane_ground_marking.size(); i++) {
	    convert(old_ros_v.hmi_lane_ground_marking[i], ros_v.hmi_lane_ground_marking[i], type);
	}
	convert(old_ros_v.hmi_lane_info_size, ros_v.hmi_lane_info_size, type);
	ros_v.hmi_lane_info.resize(old_ros_v.hmi_lane_info.size());
	for (int i = 0; i < ros_v.hmi_lane_info.size(); i++) {
	    convert(old_ros_v.hmi_lane_info[i], ros_v.hmi_lane_info[i], type);
	}
	convert(old_ros_v.hmi_obj_info_size, ros_v.hmi_obj_info_size, type);
	ros_v.hmi_obj_info.resize(old_ros_v.hmi_obj_info.size());
	for (int i = 0; i < ros_v.hmi_obj_info.size(); i++) {
	    convert(old_ros_v.hmi_obj_info[i], ros_v.hmi_obj_info[i], type);
	}
	convert(old_ros_v.cipv_track_id, ros_v.cipv_track_id, type);
	convert(old_ros_v.cutin_track_id, ros_v.cutin_track_id, type);
	convert(old_ros_v.hmi_apa_info, ros_v.hmi_apa_info, type);
	convert(old_ros_v.hmi_acc_info, ros_v.hmi_acc_info, type);
	convert(old_ros_v.hmi_scc_info, ros_v.hmi_scc_info, type);
	convert(old_ros_v.hmi_noa_info, ros_v.hmi_noa_info, type);
	convert(old_ros_v.hmi_adas_info, ros_v.hmi_adas_info, type);
	convert(old_ros_v.calib_info, ros_v.calib_info, type);
	convert(old_ros_v.sensor_info_size, ros_v.sensor_info_size, type);
	ros_v.sensor_info.resize(old_ros_v.sensor_info.size());
	for (int i = 0; i < ros_v.sensor_info.size(); i++) {
	    convert(old_ros_v.sensor_info[i], ros_v.sensor_info[i], type);
	}
	convert(old_ros_v.hmi_switch_info, ros_v.hmi_switch_info, type);
	convert(old_ros_v.hmi_hpp_output, ros_v.hmi_hpp_output, type);
	convert(old_ros_v.ehp_output, ros_v.ehp_output, type);
	convert(old_ros_v.running_mode, ros_v.running_mode, type);
	convert(old_ros_v.ground_lines_size, ros_v.ground_lines_size, type);
	ros_v.ground_lines.resize(old_ros_v.ground_lines.size());
	for (int i = 0; i < ros_v.ground_lines.size(); i++) {
	    convert(old_ros_v.ground_lines[i], ros_v.ground_lines[i], type);
	}
	convert(old_ros_v.decelers_size, ros_v.decelers_size, type);
	ros_v.decelers.resize(old_ros_v.decelers.size());
	for (int i = 0; i < ros_v.decelers.size(); i++) {
	    convert(old_ros_v.decelers[i], ros_v.decelers[i], type);
	}
	convert(old_ros_v.hmi_rads_info, ros_v.hmi_rads_info, type);
	// warning : removed struct member : RearViewMirrorCommand rear_view_mirror_signal_command
	// warning : added struct member : HmiPaInfo hmi_pa_info
	// warning : added struct member : HmiNraInfo hmi_nra_info
	// warning : added struct member : int16 hmi_sr_info
	// warning : added struct member : TurnSignalCommand hmi_turn_signal_command
	// warning : added struct member : LightSignalCommand hmi_light_signal_command
	// warning : added struct member : HornSignalCommand hmi_horn_signal_command
	// warning : added struct member : RearViewMirrorCommand hmi_rear_view_mirror_signal_command
	// warning : added struct member : uint8[40000] hmi_free_space_map
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiStopLine &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.point[i], ros_v.point[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::HmiSwitchInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.acc_switch_response, ros_v.acc_switch_response, type);
	convert(old_ros_v.lcc_switch_response, ros_v.lcc_switch_response, type);
	convert(old_ros_v.noa_switch_response, ros_v.noa_switch_response, type);
	convert(old_ros_v.apa_switch_response, ros_v.apa_switch_response, type);
	convert(old_ros_v.rpa_switch_response, ros_v.rpa_switch_response, type);
	convert(old_ros_v.hpp_switch_response, ros_v.hpp_switch_response, type);
	convert(old_ros_v.aeb_switch_response, ros_v.aeb_switch_response, type);
	convert(old_ros_v.meb_switch_response, ros_v.meb_switch_response, type);
	convert(old_ros_v.ihc_switch_response, ros_v.ihc_switch_response, type);
	convert(old_ros_v.ldw_switch_response, ros_v.ldw_switch_response, type);
	convert(old_ros_v.ldw_level_response, ros_v.ldw_level_response, type);
	convert(old_ros_v.elk_switch_response, ros_v.elk_switch_response, type);
	convert(old_ros_v.bsd_switch_response, ros_v.bsd_switch_response, type);
	convert(old_ros_v.lca_switch_response, ros_v.lca_switch_response, type);
	convert(old_ros_v.dow_switch_response, ros_v.dow_switch_response, type);
	convert(old_ros_v.fcta_switch_response, ros_v.fcta_switch_response, type);
	convert(old_ros_v.rcta_switch_response, ros_v.rcta_switch_response, type);
	convert(old_ros_v.rcw_switch_response, ros_v.rcw_switch_response, type);
	convert(old_ros_v.ldp_switch_response, ros_v.ldp_switch_response, type);
	convert(old_ros_v.fcw_switch_response, ros_v.fcw_switch_response, type);
	convert(old_ros_v.fcw_level_response, ros_v.fcw_level_response, type);
	convert(old_ros_v.speed_set_faile_rsp, ros_v.speed_set_faile_rsp, type);
	convert(old_ros_v.interval_set_fail_rsp, ros_v.interval_set_fail_rsp, type);
	convert(old_ros_v.parking_preference_faile_rsp, ros_v.parking_preference_faile_rsp, type);
	// warning : modified struct member : bool tsr_switch_response -> int16 tsr_switch_response
	convert(old_ros_v.tsr_switch_response, ros_v.tsr_switch_response, type);
	// warning : added struct member : bool fctb_switch_response
	// warning : added struct member : bool rctb_switch_response
	// warning : added struct member : bool rads_switch_response
	// warning : added struct member : bool pa_switch_response
	// warning : added struct member : bool nra_switch_response
	// warning : added struct member : bool amap_switch_response
	// warning : added struct member : bool dai_switch_response
	// warning : added struct member : bool mnp_switch_response
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Traj_Point &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.point, ros_v.point, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TrajectoryPointSet &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.point_size, ros_v.point_size, type);
	ros_v.point.resize(old_ros_v.point.size());
	for (int i = 0; i < ros_v.point.size(); i++) {
	    convert(old_ros_v.point[i], ros_v.point[i], type);
	}
}

// warning : added ros message : HmiNraInfo.msg
// warning : added ros message : HmiPaInfo.msg
// warning : added ros message : SpeedOffsetInfo.msg
REG_CONVERT_SINGLE(_iflytek_fsm_hmi_soc_outer_converter, "/iflytek/fsm/hmi_soc_outer", HmiSocOuter);
