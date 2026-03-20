#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FuncStateMachine.h"
#include "struct_msgs_v2_10/FuncStateMachine.h"
#include "struct_msgs/ParkingReq.h"
#include "struct_msgs_v2_10/ParkingReq.h"
#include "struct_msgs/PilotReq.h"
#include "struct_msgs_v2_10/PilotReq.h"
#include "struct_msgs/RadsMap.h"
#include "struct_msgs_v2_10/RadsMap.h"
#include "struct_msgs/SwitchSts.h"
#include "struct_msgs_v2_10/SwitchSts.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FuncStateMachine &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.current_state, ros_v.current_state, type);
	convert(old_ros_v.state_duration, ros_v.state_duration, type);
	convert(old_ros_v.not_available_reason, ros_v.not_available_reason, type);
	convert(old_ros_v.pilot_req, ros_v.pilot_req, type);
	convert(old_ros_v.parking_req, ros_v.parking_req, type);
	convert(old_ros_v.switch_sts, ros_v.switch_sts, type);
	convert(old_ros_v.running_mode, ros_v.running_mode, type);
	convert(old_ros_v.calib_module, ros_v.calib_module, type);
	convert(old_ros_v.system_state, ros_v.system_state, type);
	convert(old_ros_v.ehp_req, ros_v.ehp_req, type);
	convert(old_ros_v.rads_map, ros_v.rads_map, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingReq &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.apa_stop, ros_v.apa_stop, type);
	convert(old_ros_v.apa_user_preference, ros_v.apa_user_preference, type);
	convert(old_ros_v.apa_park_out_direction, ros_v.apa_park_out_direction, type);
	convert(old_ros_v.parking_select_slotid, ros_v.parking_select_slotid, type);
	convert(old_ros_v.apa_work_mode, ros_v.apa_work_mode, type);
	convert(old_ros_v.apa_parking_direction, ros_v.apa_parking_direction, type);
	convert(old_ros_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
	convert(old_ros_v.local_map_id, ros_v.local_map_id, type);
	convert(old_ros_v.pa_direction, ros_v.pa_direction, type);
	convert(old_ros_v.parking_speed_set, ros_v.parking_speed_set, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PilotReq &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_change_sts, ros_v.lane_change_sts, type);
	convert(old_ros_v.pilot_usr_preference, ros_v.pilot_usr_preference, type);
	convert(old_ros_v.acc_curise_real_spd, ros_v.acc_curise_real_spd, type);
	convert(old_ros_v.acc_curise_time_interval, ros_v.acc_curise_time_interval, type);
	convert(old_ros_v.lane_change_style, ros_v.lane_change_style, type);
	convert(old_ros_v.turn_switch, ros_v.turn_switch, type);
	convert(old_ros_v.noa_cruise_dclc_resp, ros_v.noa_cruise_dclc_resp, type);
	convert(old_ros_v.traffic_light_stop_go, ros_v.traffic_light_stop_go, type);
	convert(old_ros_v.obstacle_bypass, ros_v.obstacle_bypass, type);
	convert(old_ros_v.stand_wait, ros_v.stand_wait, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RadsMap &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.points_size, ros_v.points_size, type);
	ros_v.points.resize(old_ros_v.points.size());
	for (int i = 0; i < ros_v.points.size(); i++) {
	    convert(old_ros_v.points[i], ros_v.points[i], type);
	}
	convert(old_ros_v.length, ros_v.length, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.buffer[i], ros_v.buffer[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SwitchSts &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.acc_main_switch, ros_v.acc_main_switch, type);
	convert(old_ros_v.scc_main_switch, ros_v.scc_main_switch, type);
	convert(old_ros_v.noa_main_switch, ros_v.noa_main_switch, type);
	convert(old_ros_v.fcw_main_switch, ros_v.fcw_main_switch, type);
	convert(old_ros_v.fcw_set_sensitivity_level, ros_v.fcw_set_sensitivity_level, type);
	convert(old_ros_v.aeb_main_switch, ros_v.aeb_main_switch, type);
	convert(old_ros_v.meb_main_switch, ros_v.meb_main_switch, type);
	convert(old_ros_v.ihc_main_switch, ros_v.ihc_main_switch, type);
	convert(old_ros_v.ldw_main_switch, ros_v.ldw_main_switch, type);
	convert(old_ros_v.ldw_set_sensitivity_level, ros_v.ldw_set_sensitivity_level, type);
	convert(old_ros_v.elk_main_switch, ros_v.elk_main_switch, type);
	convert(old_ros_v.bsd_main_switch, ros_v.bsd_main_switch, type);
	convert(old_ros_v.lca_main_switch, ros_v.lca_main_switch, type);
	convert(old_ros_v.dow_main_switch, ros_v.dow_main_switch, type);
	convert(old_ros_v.fcta_main_switch, ros_v.fcta_main_switch, type);
	convert(old_ros_v.rcta_main_switch, ros_v.rcta_main_switch, type);
	convert(old_ros_v.rcw_main_switch, ros_v.rcw_main_switch, type);
	convert(old_ros_v.ldp_main_switch, ros_v.ldp_main_switch, type);
	convert(old_ros_v.apa_main_switch, ros_v.apa_main_switch, type);
	convert(old_ros_v.hpp_main_switch, ros_v.hpp_main_switch, type);
	convert(old_ros_v.rads_main_switch, ros_v.rads_main_switch, type);
	convert(old_ros_v.fctb_main_switch, ros_v.fctb_main_switch, type);
	convert(old_ros_v.rctb_main_switch, ros_v.rctb_main_switch, type);
	convert(old_ros_v.amap_main_switch, ros_v.amap_main_switch, type);
	convert(old_ros_v.dai_main_switch, ros_v.dai_main_switch, type);
	convert(old_ros_v.tsr_main_switch, ros_v.tsr_main_switch, type);
}

REG_CONVERT_SINGLE(_iflytek_fsm_soc_state_converter, "/iflytek/fsm/soc_state", FuncStateMachine);
