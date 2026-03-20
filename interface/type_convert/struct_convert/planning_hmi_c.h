#pragma once

#include "base_convert.h"
#include "c/planning_hmi_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/camera_perception_tsr_c.h"
#include "struct_convert/fusion_road_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::LDWOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ldw_state, ros_v.ldw_state, type);
  convert(struct_v.ldw_left_warning, ros_v.ldw_left_warning, type);
  convert(struct_v.ldw_right_warning, ros_v.ldw_right_warning, type);
}

template <typename T2>
void convert(iflyauto::LDPOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ldp_state, ros_v.ldp_state, type);
  convert(struct_v.ldp_left_intervention_flag, ros_v.ldp_left_intervention_flag, type);
  convert(struct_v.ldp_right_intervention_flag, ros_v.ldp_right_intervention_flag, type);
  convert(struct_v.ldp_warning_audio_flag, ros_v.ldp_warning_audio_flag, type);
  convert(struct_v.ldp_driver_handsoff_warning, ros_v.ldp_driver_handsoff_warning, type);
}

template <typename T2>
void convert(iflyauto::ELKRiskObjInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_valid, ros_v.obj_valid, type);
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.light_type, ros_v.light_type, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.relative_velocity, ros_v.relative_velocity, type);
  convert(struct_v.relative_acceleration, ros_v.relative_acceleration, type);
  convert(struct_v.relative_center_position, ros_v.relative_center_position, type);
  convert(struct_v.relative_heading_angle, ros_v.relative_heading_angle, type);
  convert(struct_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
}

template <typename T2>
void convert(iflyauto::ELKOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.elk_state, ros_v.elk_state, type);
  convert(struct_v.elk_left_intervention_flag, ros_v.elk_left_intervention_flag, type);
  convert(struct_v.elk_right_intervention_flag, ros_v.elk_right_intervention_flag, type);
  convert(struct_v.elk_risk_obj, ros_v.elk_risk_obj, type);
}

template <typename T2>
void convert(iflyauto::TSROutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tsr_state, ros_v.tsr_state, type);
  convert(struct_v.tsr_speed_limit, ros_v.tsr_speed_limit, type);
  convert(struct_v.tsr_warning, ros_v.tsr_warning, type);
  convert(struct_v.tsr_supp_sign_type, ros_v.tsr_supp_sign_type, type);
  convert(struct_v.tsr_speed_unlimit_warning, ros_v.tsr_speed_unlimit_warning, type);
}

template <typename T2>
void convert(iflyauto::IHCOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ihc_state, ros_v.ihc_state, type);
  convert(struct_v.ihc_request_status, ros_v.ihc_request_status, type);
  convert(struct_v.ihc_request, ros_v.ihc_request, type);
}

template <typename T2>
void convert(iflyauto::AMAPOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.amap_state, ros_v.amap_state, type);
  convert(struct_v.amap_request_flag, ros_v.amap_request_flag, type);
  convert(struct_v.amap_trq_limit_max, ros_v.amap_trq_limit_max, type);
}

template <typename T2>
void convert(iflyauto::MEBOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.meb_state, ros_v.meb_state, type);
  convert(struct_v.meb_request_status, ros_v.meb_request_status, type);
  convert(struct_v.meb_request_value, ros_v.meb_request_value, type);
  convert(struct_v.meb_request_direction, ros_v.meb_request_direction, type);
}

template <typename T2>
void convert(iflyauto::ALCOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lc_request_size, ros_v.lc_request_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lc_request_size >= 0 && struct_v.lc_request_size <= PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      ros_v.lc_request.resize(struct_v.lc_request_size);
    } else {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lc_request_size=" << struct_v.lc_request_size 
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
      ros_v.lc_request_size = PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM;
      ros_v.lc_request.resize(PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lc_request.size(); i0++) {
      convert(struct_v.lc_request[i0], ros_v.lc_request[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lc_request_size > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM || ros_v.lc_request_size < 0 || ros_v.lc_request.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lc_request_size=" << ros_v.lc_request_size 
                << " ros_v.lc_request.size()=" << ros_v.lc_request.size()
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lc_request.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      for (size_t i0 = 0; i0 < PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM; i0++) {
        convert(struct_v.lc_request[i0], ros_v.lc_request[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lc_request.size(); i0++) {
        convert(struct_v.lc_request[i0], ros_v.lc_request[i0], type);
      }
    }
  }
  //
  convert(struct_v.lc_status_size, ros_v.lc_status_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lc_status_size >= 0 && struct_v.lc_status_size <= PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      ros_v.lc_status.resize(struct_v.lc_status_size);
    } else {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lc_status_size=" << struct_v.lc_status_size 
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
      ros_v.lc_status_size = PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM;
      ros_v.lc_status.resize(PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.lc_status.size(); i1++) {
      convert(struct_v.lc_status[i1], ros_v.lc_status[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lc_status_size > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM || ros_v.lc_status_size < 0 || ros_v.lc_status.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lc_status_size=" << ros_v.lc_status_size 
                << " ros_v.lc_status.size()=" << ros_v.lc_status.size()
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lc_status.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      for (size_t i1 = 0; i1 < PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM; i1++) {
        convert(struct_v.lc_status[i1], ros_v.lc_status[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lc_status.size(); i1++) {
        convert(struct_v.lc_status[i1], ros_v.lc_status[i1], type);
      }
    }
  }
  //
  convert(struct_v.lc_invalid_reason_size, ros_v.lc_invalid_reason_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lc_invalid_reason_size >= 0 && struct_v.lc_invalid_reason_size <= PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      ros_v.lc_invalid_reason.resize(struct_v.lc_invalid_reason_size);
    } else {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lc_invalid_reason_size=" << struct_v.lc_invalid_reason_size 
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
      ros_v.lc_invalid_reason_size = PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM;
      ros_v.lc_invalid_reason.resize(PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.lc_invalid_reason.size(); i2++) {
      convert(struct_v.lc_invalid_reason[i2], ros_v.lc_invalid_reason[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lc_invalid_reason_size > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM || ros_v.lc_invalid_reason_size < 0 || ros_v.lc_invalid_reason.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lc_invalid_reason_size=" << ros_v.lc_invalid_reason_size 
                << " ros_v.lc_invalid_reason.size()=" << ros_v.lc_invalid_reason.size()
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lc_invalid_reason.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      for (size_t i2 = 0; i2 < PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM; i2++) {
        convert(struct_v.lc_invalid_reason[i2], ros_v.lc_invalid_reason[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.lc_invalid_reason.size(); i2++) {
        convert(struct_v.lc_invalid_reason[i2], ros_v.lc_invalid_reason[i2], type);
      }
    }
  }
  //
  convert(struct_v.lc_back_reason_size, ros_v.lc_back_reason_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lc_back_reason_size >= 0 && struct_v.lc_back_reason_size <= PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      ros_v.lc_back_reason.resize(struct_v.lc_back_reason_size);
    } else {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lc_back_reason_size=" << struct_v.lc_back_reason_size 
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
      ros_v.lc_back_reason_size = PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM;
      ros_v.lc_back_reason.resize(PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.lc_back_reason.size(); i3++) {
      convert(struct_v.lc_back_reason[i3], ros_v.lc_back_reason[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lc_back_reason_size > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM || ros_v.lc_back_reason_size < 0 || ros_v.lc_back_reason.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lc_back_reason_size=" << ros_v.lc_back_reason_size 
                << " ros_v.lc_back_reason.size()=" << ros_v.lc_back_reason.size()
                << " not in range PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM=" << PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lc_back_reason.size() > PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM) {
      for (size_t i3 = 0; i3 < PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM; i3++) {
        convert(struct_v.lc_back_reason[i3], ros_v.lc_back_reason[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.lc_back_reason.size(); i3++) {
        convert(struct_v.lc_back_reason[i3], ros_v.lc_back_reason[i3], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::CIPVInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.has_cipv, ros_v.has_cipv, type);
  convert(struct_v.cipv_id, ros_v.cipv_id, type);
}

template <typename T2>
void convert(iflyauto::TLAOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.traffic_light_reminder, ros_v.traffic_light_reminder, type);
}

template <typename T2>
void convert(iflyauto::LandingPoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_avaliable, ros_v.is_avaliable, type);
  convert(struct_v.relative_pos, ros_v.relative_pos, type);
  convert(struct_v.heading, ros_v.heading, type);
}

template <typename T2>
void convert(iflyauto::LCCS_TrajPoly &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.c0, ros_v.c0, type);
  convert(struct_v.c1, ros_v.c1, type);
  convert(struct_v.c2, ros_v.c2, type);
  convert(struct_v.c3, ros_v.c3, type);
  convert(struct_v.c4, ros_v.c4, type);
  convert(struct_v.c5, ros_v.c5, type);
  convert(struct_v.start_x, ros_v.start_x, type);
  convert(struct_v.end_x, ros_v.end_x, type);
}

template <typename T2>
void convert(iflyauto::LCCSTrajectoryPoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.start, ros_v.start, type);
  convert(struct_v.middle, ros_v.middle, type);
  convert(struct_v.end, ros_v.end, type);
}

template <typename T2>
void convert(iflyauto::ObstacleInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.speed_x, ros_v.speed_x, type);
  convert(struct_v.speed_y, ros_v.speed_y, type);
  convert(struct_v.heading, ros_v.heading, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.center_x, ros_v.center_x, type);
  convert(struct_v.center_y, ros_v.center_y, type);
  convert(struct_v.size, ros_v.size, type);
  convert(struct_v.lon_status, ros_v.lon_status, type);
  convert(struct_v.anchor, ros_v.anchor, type);
}

template <typename T2>
void convert(iflyauto::ConstructionInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.construction_state, ros_v.construction_state, type);
  convert(struct_v.distance_to_construction, ros_v.distance_to_construction, type);
}

template <typename T2>
void convert(iflyauto::ConeWarningInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.cone_warning, ros_v.cone_warning, type);
  convert(struct_v.cone_warning_action, ros_v.cone_warning_action, type);
}

template <typename T2>
void convert(iflyauto::AD2HMIData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_avaliable, ros_v.is_avaliable, type);
  convert(struct_v.timestamp, ros_v.timestamp, type);
  convert(struct_v.lane_change_direction, ros_v.lane_change_direction, type);
  convert(struct_v.lane_change_status, ros_v.lane_change_status, type);
  convert(struct_v.lane_change_reason, ros_v.lane_change_reason, type);
  convert(struct_v.status_update_reason, ros_v.status_update_reason, type);
  convert(struct_v.noa_exit_warning_level_distance, ros_v.noa_exit_warning_level_distance, type);
  convert(struct_v.avoid_status, ros_v.avoid_status, type);
  convert(struct_v.aovid_id, ros_v.aovid_id, type);
  convert(struct_v.is_curva, ros_v.is_curva, type);
  convert(struct_v.landing_point, ros_v.landing_point, type);
  convert(struct_v.tp, ros_v.tp, type);
  convert(struct_v.obstacle_info_size, ros_v.obstacle_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.obstacle_info_size >= 0 && struct_v.obstacle_info_size <= PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM) {
      ros_v.obstacle_info.resize(struct_v.obstacle_info_size);
    } else {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << " [convert][TO_ROS] obstacle_info_size=" << struct_v.obstacle_info_size 
                << " not in range PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM=" << PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM 
                << std::endl;
      ros_v.obstacle_info_size = PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM;
      ros_v.obstacle_info.resize(PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.obstacle_info.size(); i0++) {
      convert(struct_v.obstacle_info[i0], ros_v.obstacle_info[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.obstacle_info_size > PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM || ros_v.obstacle_info_size < 0 || ros_v.obstacle_info.size() > PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM) {
      std::cout << "convert/planning_hmi_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] obstacle_info_size=" << ros_v.obstacle_info_size 
                << " ros_v.obstacle_info.size()=" << ros_v.obstacle_info.size()
                << " not in range PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM=" << PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.obstacle_info.size() > PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM) {
      for (size_t i0 = 0; i0 < PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM; i0++) {
        convert(struct_v.obstacle_info[i0], ros_v.obstacle_info[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.obstacle_info.size(); i0++) {
        convert(struct_v.obstacle_info[i0], ros_v.obstacle_info[i0], type);
      }
    }
  }
  //
  convert(struct_v.cutin_track_id, ros_v.cutin_track_id, type);
  convert(struct_v.cutin_ttc, ros_v.cutin_ttc, type);
  convert(struct_v.cruise_speed, ros_v.cruise_speed, type);
  convert(struct_v.avoiddirect, ros_v.avoiddirect, type);
  convert(struct_v.distance_to_ramp, ros_v.distance_to_ramp, type);
  convert(struct_v.distance_to_split, ros_v.distance_to_split, type);
  convert(struct_v.distance_to_merge, ros_v.distance_to_merge, type);
  convert(struct_v.distance_to_toll_station, ros_v.distance_to_toll_station, type);
  convert(struct_v.distance_to_tunnel, ros_v.distance_to_tunnel, type);
  convert(struct_v.is_within_hdmap, ros_v.is_within_hdmap, type);
  convert(struct_v.ramp_direction, ros_v.ramp_direction, type);
  convert(struct_v.ramp_pass_sts, ros_v.ramp_pass_sts, type);
  convert(struct_v.intersection_pass_sts, ros_v.intersection_pass_sts, type);
  convert(struct_v.dis_to_reference_line, ros_v.dis_to_reference_line, type);
  convert(struct_v.angle_to_roaddirection, ros_v.angle_to_roaddirection, type);
  convert(struct_v.is_in_sdmaproad, ros_v.is_in_sdmaproad, type);
  convert(struct_v.road_type, ros_v.road_type, type);
  convert(struct_v.reference_line_msg, ros_v.reference_line_msg, type);
  convert(struct_v.borrow_lane_type, ros_v.borrow_lane_type, type);
  convert(struct_v.borrow_direction, ros_v.borrow_direction, type);
  convert(struct_v.borrow_failed_reason, ros_v.borrow_failed_reason, type);
  convert(struct_v.borrow_target, ros_v.borrow_target, type);
  convert(struct_v.start_nudging, ros_v.start_nudging, type);
  convert(struct_v.cone_warning_info, ros_v.cone_warning_info, type);
  convert(struct_v.construction_info, ros_v.construction_info, type);
  convert(struct_v.intersection_state, ros_v.intersection_state, type);
  convert(struct_v.traffic_light_reminder, ros_v.traffic_light_reminder, type);
  convert(struct_v.split_select_direction, ros_v.split_select_direction, type);
}

template <typename T2>
void convert(iflyauto::HPPHMIData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_avaliable, ros_v.is_avaliable, type);
  convert(struct_v.distance_to_parking_space, ros_v.distance_to_parking_space, type);
  convert(struct_v.avoid_status, ros_v.avoid_status, type);
  convert(struct_v.avoid_obstacle_type, ros_v.avoid_obstacle_type, type);
  convert(struct_v.aovid_id, ros_v.aovid_id, type);
  convert(struct_v.is_approaching_turn, ros_v.is_approaching_turn, type);
  convert(struct_v.is_left_turn, ros_v.is_left_turn, type);
  convert(struct_v.is_approaching_intersection, ros_v.is_approaching_intersection, type);
  convert(struct_v.is_approaching_speed_bumps, ros_v.is_approaching_speed_bumps, type);
  convert(struct_v.emergency_level, ros_v.emergency_level, type);
  convert(struct_v.is_target_parking_space_occupied, ros_v.is_target_parking_space_occupied, type);
  convert(struct_v.is_new_parking_space_found, ros_v.is_new_parking_space_found, type);
  convert(struct_v.is_on_hpp_lane, ros_v.is_on_hpp_lane, type);
  convert(struct_v.is_reached_hpp_trace_start, ros_v.is_reached_hpp_trace_start, type);
  convert(struct_v.accumulated_driving_distance, ros_v.accumulated_driving_distance, type);
  convert(struct_v.estimated_remaining_time, ros_v.estimated_remaining_time, type);
  convert(struct_v.hpp_state_switch, ros_v.hpp_state_switch, type);
  convert(struct_v.hpp_planning_failed_reason, ros_v.hpp_planning_failed_reason, type);
}

template <typename T2>
void convert(iflyauto::APAHMIData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.remain_dist, ros_v.remain_dist, type);
  convert(struct_v.is_parking_pause, ros_v.is_parking_pause, type);
  convert(struct_v.parking_pause_reason, ros_v.parking_pause_reason, type);
  convert(struct_v.prepare_plan_state, ros_v.prepare_plan_state, type);
  convert(struct_v.planning_park_dir, ros_v.planning_park_dir, type);
  convert(struct_v.planning_recommend_park_dir, ros_v.planning_recommend_park_dir, type);
  convert(struct_v.recommend_park_out, ros_v.recommend_park_out, type);
  convert(struct_v.recommend_park_in, ros_v.recommend_park_in, type);
  convert(struct_v.pa_remain_distance, ros_v.pa_remain_distance, type);
  convert(struct_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
  convert(struct_v.planning_park_pa_dir, ros_v.planning_park_pa_dir, type);
  convert(struct_v.planning_recommend_pa_dir, ros_v.planning_recommend_pa_dir, type);
}

template <typename T2>
void convert(iflyauto::NSAHMIData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_avaliable, ros_v.is_avaliable, type);
  convert(struct_v.nsa_disable_reason, ros_v.nsa_disable_reason, type);
  convert(struct_v.is_complete, ros_v.is_complete, type);
  convert(struct_v.nsa_complete_reason, ros_v.nsa_complete_reason, type);
  convert(struct_v.nsa_pause_reason, ros_v.nsa_pause_reason, type);
}

template <typename T2>
void convert(iflyauto::RADSHMIData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.is_avaliable, ros_v.is_avaliable, type);
  convert(struct_v.rads_pause_reason, ros_v.rads_pause_reason, type);
  convert(struct_v.avoid_status, ros_v.avoid_status, type);
  convert(struct_v.aovid_id, ros_v.aovid_id, type);
}

template <typename T2>
void convert(iflyauto::PlanningHMIOutputInfoStr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.ldw_output_info, ros_v.ldw_output_info, type);
  convert(struct_v.ldp_output_info, ros_v.ldp_output_info, type);
  convert(struct_v.elk_output_info, ros_v.elk_output_info, type);
  convert(struct_v.tsr_output_info, ros_v.tsr_output_info, type);
  convert(struct_v.ihc_output_info, ros_v.ihc_output_info, type);
  convert(struct_v.amap_output_info, ros_v.amap_output_info, type);
  convert(struct_v.meb_output_info, ros_v.meb_output_info, type);
  convert(struct_v.alc_output_info, ros_v.alc_output_info, type);
  convert(struct_v.cipv_info, ros_v.cipv_info, type);
  convert(struct_v.tla_output_info, ros_v.tla_output_info, type);
  convert(struct_v.ad_info, ros_v.ad_info, type);
  convert(struct_v.hpp_info, ros_v.hpp_info, type);
  convert(struct_v.apa_info, ros_v.apa_info, type);
  convert(struct_v.nsa_info, ros_v.nsa_info, type);
  convert(struct_v.rads_info, ros_v.rads_info, type);
}

