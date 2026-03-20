#pragma once

#include "base_convert.h"
#include "c/func_state_machine_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/planning_hmi_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/hmi_inner_c.h"
#include "struct_convert/ifly_parking_map_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::SwitchSts &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acc_main_switch, ros_v.acc_main_switch, type);
  convert(struct_v.scc_main_switch, ros_v.scc_main_switch, type);
  convert(struct_v.noa_main_switch, ros_v.noa_main_switch, type);
  convert(struct_v.fcw_main_switch, ros_v.fcw_main_switch, type);
  convert(struct_v.fcw_set_sensitivity_level, ros_v.fcw_set_sensitivity_level, type);
  convert(struct_v.aeb_main_switch, ros_v.aeb_main_switch, type);
  convert(struct_v.meb_main_switch, ros_v.meb_main_switch, type);
  convert(struct_v.tsr_main_switch, ros_v.tsr_main_switch, type);
  convert(struct_v.ihc_main_switch, ros_v.ihc_main_switch, type);
  convert(struct_v.ldw_main_switch, ros_v.ldw_main_switch, type);
  convert(struct_v.ldw_set_sensitivity_level, ros_v.ldw_set_sensitivity_level, type);
  convert(struct_v.elk_main_switch, ros_v.elk_main_switch, type);
  convert(struct_v.bsd_main_switch, ros_v.bsd_main_switch, type);
  convert(struct_v.lca_main_switch, ros_v.lca_main_switch, type);
  convert(struct_v.dow_main_switch, ros_v.dow_main_switch, type);
  convert(struct_v.fcta_main_switch, ros_v.fcta_main_switch, type);
  convert(struct_v.fctb_main_switch, ros_v.fctb_main_switch, type);
  convert(struct_v.rcta_main_switch, ros_v.rcta_main_switch, type);
  convert(struct_v.rctb_main_switch, ros_v.rctb_main_switch, type);
  convert(struct_v.rcw_main_switch, ros_v.rcw_main_switch, type);
  convert(struct_v.ldp_main_switch, ros_v.ldp_main_switch, type);
  convert(struct_v.apa_main_switch, ros_v.apa_main_switch, type);
  convert(struct_v.hpp_main_switch, ros_v.hpp_main_switch, type);
  convert(struct_v.rads_main_switch, ros_v.rads_main_switch, type);
  convert(struct_v.amap_main_switch, ros_v.amap_main_switch, type);
  convert(struct_v.dai_main_switch, ros_v.dai_main_switch, type);
  convert(struct_v.dow_secondary_alert_main_switch, ros_v.dow_secondary_alert_main_switch, type);
  convert(struct_v.blue_light_main_switch, ros_v.blue_light_main_switch, type);
}

template <typename T2>
void convert(iflyauto::ParkingReq &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_stop, ros_v.apa_stop, type);
  convert(struct_v.apa_user_preference, ros_v.apa_user_preference, type);
  convert(struct_v.apa_park_out_direction, ros_v.apa_park_out_direction, type);
  convert(struct_v.parking_select_slotid, ros_v.parking_select_slotid, type);
  convert(struct_v.apa_work_mode, ros_v.apa_work_mode, type);
  convert(struct_v.apa_parking_direction, ros_v.apa_parking_direction, type);
  convert(struct_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
  convert(struct_v.local_map_id, ros_v.local_map_id, type);
  convert(struct_v.pa_direction, ros_v.pa_direction, type);
  convert(struct_v.parking_speed_set, ros_v.parking_speed_set, type);
  convert(struct_v.rear_view_mirror_signal_command, ros_v.rear_view_mirror_signal_command, type);
}

template <typename T2>
void convert(iflyauto::HPPReq &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.need_gnss_loss_signal, ros_v.need_gnss_loss_signal, type);
}

template <typename T2>
void convert(iflyauto::PilotReq &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_change_sts, ros_v.lane_change_sts, type);
  convert(struct_v.pilot_usr_preference, ros_v.pilot_usr_preference, type);
  convert(struct_v.acc_curise_real_spd, ros_v.acc_curise_real_spd, type);
  convert(struct_v.acc_curise_time_interval, ros_v.acc_curise_time_interval, type);
  convert(struct_v.lane_change_style, ros_v.lane_change_style, type);
  convert(struct_v.noa_cruise_dclc_resp, ros_v.noa_cruise_dclc_resp, type);
  convert(struct_v.is_overtake_lane_change_confirmed, ros_v.is_overtake_lane_change_confirmed, type);
  convert(struct_v.traffic_light_stop_go, ros_v.traffic_light_stop_go, type);
  convert(struct_v.obstacle_bypass, ros_v.obstacle_bypass, type);
  convert(struct_v.stand_wait, ros_v.stand_wait, type);
  convert(struct_v.turn_switch, ros_v.turn_switch, type);
  convert(struct_v.has_obstacle_ahead, ros_v.has_obstacle_ahead, type);
}

template <typename T2>
void convert(iflyauto::RadsMap &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.points_size, ros_v.points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.points_size >= 0 && struct_v.points_size <= RADS_MAP_POINT_MAX_NUM) {
      ros_v.points.resize(struct_v.points_size);
    } else {
      std::cout << "convert/func_state_machine_c.h:" << __LINE__ 
                << " [convert][TO_ROS] points_size=" << struct_v.points_size 
                << " not in range RADS_MAP_POINT_MAX_NUM=" << RADS_MAP_POINT_MAX_NUM 
                << std::endl;
      ros_v.points_size = RADS_MAP_POINT_MAX_NUM;
      ros_v.points.resize(RADS_MAP_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.points.size(); i0++) {
      convert(struct_v.points[i0], ros_v.points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.points_size > RADS_MAP_POINT_MAX_NUM || ros_v.points_size < 0 || ros_v.points.size() > RADS_MAP_POINT_MAX_NUM) {
      std::cout << "convert/func_state_machine_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] points_size=" << ros_v.points_size 
                << " ros_v.points.size()=" << ros_v.points.size()
                << " not in range RADS_MAP_POINT_MAX_NUM=" << RADS_MAP_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.points.size() > RADS_MAP_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < RADS_MAP_POINT_MAX_NUM; i0++) {
        convert(struct_v.points[i0], ros_v.points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.points.size(); i0++) {
        convert(struct_v.points[i0], ros_v.points[i0], type);
      }
    }
  }
  //
  convert(struct_v.length, ros_v.length, type);
  for (size_t i1 = 0; i1 < ros_v.buffer.size(); i1++) {
	  convert(struct_v.buffer[i1], ros_v.buffer[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::NraReq &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.nra_distance, ros_v.nra_distance, type);
}

template <typename T2>
void convert(iflyauto::FuncStateMachine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.current_state, ros_v.current_state, type);
  convert(struct_v.background_state, ros_v.background_state, type);
  convert(struct_v.state_duration, ros_v.state_duration, type);
  convert(struct_v.not_available_reason, ros_v.not_available_reason, type);
  convert(struct_v.pilot_req, ros_v.pilot_req, type);
  convert(struct_v.parking_req, ros_v.parking_req, type);
  convert(struct_v.hpp_req, ros_v.hpp_req, type);
  convert(struct_v.switch_sts, ros_v.switch_sts, type);
  convert(struct_v.running_mode, ros_v.running_mode, type);
  convert(struct_v.calib_module, ros_v.calib_module, type);
  convert(struct_v.system_state, ros_v.system_state, type);
  convert(struct_v.ehp_req, ros_v.ehp_req, type);
  convert(struct_v.rads_map, ros_v.rads_map, type);
  convert(struct_v.nra_req, ros_v.nra_req, type);
}

