#pragma once

#include "base_convert.h"
#include "c/hmi_soc_outer_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/hmi_inner_c.h"
#include "struct_convert/planning_hmi_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_convert/fusion_objects_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/ifly_parking_map_c.h"
#include "struct_convert/fusion_road_c.h"
#include "struct_convert/camera_perception_lane_lines_c.h"
#include "struct_convert/camera_perception_tsr_c.h"
#include "struct_convert/camera_perception_groundline_c.h"
#include "struct_convert/camera_perception_deceler_c.h"
#include "struct_convert/ifly_localization_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::TrajectoryPointSet &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.point_size, ros_v.point_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.point_size >= 0 && struct_v.point_size <= PLANNING_TRAJ_POINTS_MAX_NUM) {
      ros_v.point.resize(struct_v.point_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] point_size=" << struct_v.point_size 
                << " not in range PLANNING_TRAJ_POINTS_MAX_NUM=" << PLANNING_TRAJ_POINTS_MAX_NUM 
                << std::endl;
      ros_v.point_size = PLANNING_TRAJ_POINTS_MAX_NUM;
      ros_v.point.resize(PLANNING_TRAJ_POINTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.point.size(); i0++) {
      convert(struct_v.point[i0], ros_v.point[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.point_size > PLANNING_TRAJ_POINTS_MAX_NUM || ros_v.point_size < 0 || ros_v.point.size() > PLANNING_TRAJ_POINTS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] point_size=" << ros_v.point_size 
                << " ros_v.point.size()=" << ros_v.point.size()
                << " not in range PLANNING_TRAJ_POINTS_MAX_NUM=" << PLANNING_TRAJ_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.point.size() > PLANNING_TRAJ_POINTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < PLANNING_TRAJ_POINTS_MAX_NUM; i0++) {
        convert(struct_v.point[i0], ros_v.point[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.point.size(); i0++) {
        convert(struct_v.point[i0], ros_v.point[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::HmiCommon &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.common_notify_req, ros_v.common_notify_req, type);
  convert(struct_v.vehicle_speed_display, ros_v.vehicle_speed_display, type);
  convert(struct_v.brake_pedal_pressed, ros_v.brake_pedal_pressed, type);
  convert(struct_v.shift_lever_state, ros_v.shift_lever_state, type);
  convert(struct_v.turn_light_state, ros_v.turn_light_state, type);
  convert(struct_v.tp_set, ros_v.tp_set, type);
  convert(struct_v.position, ros_v.position, type);
  convert(struct_v.rear_view_mirror_signal_command, ros_v.rear_view_mirror_signal_command, type);
  convert(struct_v.adas_takeover_req, ros_v.adas_takeover_req, type);
  convert(struct_v.adas_takeover_reason, ros_v.adas_takeover_reason, type);
  convert(struct_v.CruiseAccelerateSts, ros_v.CruiseAccelerateSts, type);
  convert(struct_v.avoiddirect, ros_v.avoiddirect, type);
  convert(struct_v.borrow_direction, ros_v.borrow_direction, type);
}

template <typename T2>
void convert(iflyauto::HmiNoaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.noa_status, ros_v.noa_status, type);
  convert(struct_v.noa_activate_resp, ros_v.noa_activate_resp, type);
  convert(struct_v.noa_driver_denied, ros_v.noa_driver_denied, type);
  convert(struct_v.noa_takeover_req_lv, ros_v.noa_takeover_req_lv, type);
  convert(struct_v.distance_to_destination, ros_v.distance_to_destination, type);
  convert(struct_v.distance_to_station, ros_v.distance_to_station, type);
  convert(struct_v.distance_to_ramp, ros_v.distance_to_ramp, type);
  convert(struct_v.distance_to_tunnel, ros_v.distance_to_tunnel, type);
  convert(struct_v.distance_to_split, ros_v.distance_to_split, type);
  convert(struct_v.distance_to_merge, ros_v.distance_to_merge, type);
  convert(struct_v.noa_notify_req, ros_v.noa_notify_req, type);
  convert(struct_v.noa_notify, ros_v.noa_notify, type);
  convert(struct_v.lane_change_style, ros_v.lane_change_style, type);
  convert(struct_v.noa_cruise_dclc_resp, ros_v.noa_cruise_dclc_resp, type);
}

template <typename T2>
void convert(iflyauto::HmiLineInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.line_id, ros_v.line_id, type);
  convert(struct_v.line_index, ros_v.line_index, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.line_segments_size, ros_v.line_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.line_segments_size >= 0 && struct_v.line_segments_size <= CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      ros_v.line_segments.resize(struct_v.line_segments_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] line_segments_size=" << struct_v.line_segments_size 
                << " not in range CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.line_segments_size = CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM;
      ros_v.line_segments.resize(CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.line_segments.size(); i0++) {
      convert(struct_v.line_segments[i0], ros_v.line_segments[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.line_segments_size > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM || ros_v.line_segments_size < 0 || ros_v.line_segments.size() > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] line_segments_size=" << ros_v.line_segments_size 
                << " ros_v.line_segments.size()=" << ros_v.line_segments.size()
                << " not in range CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.line_segments.size() > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM; i0++) {
        convert(struct_v.line_segments[i0], ros_v.line_segments[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.line_segments.size(); i0++) {
        convert(struct_v.line_segments[i0], ros_v.line_segments[i0], type);
      }
    }
  }
  //
  convert(struct_v.marking_segments_size, ros_v.marking_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.marking_segments_size >= 0 && struct_v.marking_segments_size <= CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      ros_v.marking_segments.resize(struct_v.marking_segments_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] marking_segments_size=" << struct_v.marking_segments_size 
                << " not in range CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.marking_segments_size = CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM;
      ros_v.marking_segments.resize(CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.marking_segments.size(); i1++) {
      convert(struct_v.marking_segments[i1], ros_v.marking_segments[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.marking_segments_size > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM || ros_v.marking_segments_size < 0 || ros_v.marking_segments.size() > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] marking_segments_size=" << ros_v.marking_segments_size 
                << " ros_v.marking_segments.size()=" << ros_v.marking_segments.size()
                << " not in range CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.marking_segments.size() > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM; i1++) {
        convert(struct_v.marking_segments[i1], ros_v.marking_segments[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.marking_segments.size(); i1++) {
        convert(struct_v.marking_segments[i1], ros_v.marking_segments[i1], type);
      }
    }
  }
  //
  convert(struct_v.color_segments_size, ros_v.color_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.color_segments_size >= 0 && struct_v.color_segments_size <= CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      ros_v.color_segments.resize(struct_v.color_segments_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] color_segments_size=" << struct_v.color_segments_size 
                << " not in range CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.color_segments_size = CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM;
      ros_v.color_segments.resize(CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.color_segments.size(); i2++) {
      convert(struct_v.color_segments[i2], ros_v.color_segments[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.color_segments_size > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM || ros_v.color_segments_size < 0 || ros_v.color_segments.size() > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] color_segments_size=" << ros_v.color_segments_size 
                << " ros_v.color_segments.size()=" << ros_v.color_segments.size()
                << " not in range CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.color_segments.size() > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM; i2++) {
        convert(struct_v.color_segments[i2], ros_v.color_segments[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.color_segments.size(); i2++) {
        convert(struct_v.color_segments[i2], ros_v.color_segments[i2], type);
      }
    }
  }
  //
  convert(struct_v.car_points_size, ros_v.car_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.car_points_size >= 0 && struct_v.car_points_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.car_points.resize(struct_v.car_points_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] car_points_size=" << struct_v.car_points_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.car_points_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.car_points.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.car_points.size(); i3++) {
      convert(struct_v.car_points[i3], ros_v.car_points[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.car_points_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.car_points_size < 0 || ros_v.car_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] car_points_size=" << ros_v.car_points_size 
                << " ros_v.car_points.size()=" << ros_v.car_points.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.car_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i3 = 0; i3 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i3++) {
        convert(struct_v.car_points[i3], ros_v.car_points[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.car_points.size(); i3++) {
        convert(struct_v.car_points[i3], ros_v.car_points[i3], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::HmiObjInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.target_long_position, ros_v.target_long_position, type);
  convert(struct_v.target_lat_position, ros_v.target_lat_position, type);
  convert(struct_v.heading_angle, ros_v.heading_angle, type);
  convert(struct_v.target_track_id, ros_v.target_track_id, type);
  convert(struct_v.target_type, ros_v.target_type, type);
  convert(struct_v.motion_pattern, ros_v.motion_pattern, type);
  convert(struct_v.light_status, ros_v.light_status, type);
  convert(struct_v.shape, ros_v.shape, type);
  convert(struct_v.velocity, ros_v.velocity, type);
  convert(struct_v.lane_id, ros_v.lane_id, type);
}

template <typename T2>
void convert(iflyauto::HmiApaSlotInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_slot_id, ros_v.apa_slot_id, type);
  convert(struct_v.apa_slot_type, ros_v.apa_slot_type, type);
  for (size_t i0 = 0; i0 < ros_v.apa_slot_corner_points.size(); i0++) {
	  convert(struct_v.apa_slot_corner_points[i0], ros_v.apa_slot_corner_points[i0], type);
  }
  convert(struct_v.allow_parking, ros_v.allow_parking, type);
  convert(struct_v.planning_success, ros_v.planning_success, type);
  convert(struct_v.is_narrow_slot, ros_v.is_narrow_slot, type);
  convert(struct_v.apa_slot_num, ros_v.apa_slot_num, type);
  convert(struct_v.limiters_size, ros_v.limiters_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.limiters_size >= 0 && struct_v.limiters_size <= FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      ros_v.limiters.resize(struct_v.limiters_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] limiters_size=" << struct_v.limiters_size 
                << " not in range FUSION_PARKING_SLOT_LIMITER_MAX_NUM=" << FUSION_PARKING_SLOT_LIMITER_MAX_NUM 
                << std::endl;
      ros_v.limiters_size = FUSION_PARKING_SLOT_LIMITER_MAX_NUM;
      ros_v.limiters.resize(FUSION_PARKING_SLOT_LIMITER_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.limiters.size(); i1++) {
      convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.limiters_size > FUSION_PARKING_SLOT_LIMITER_MAX_NUM || ros_v.limiters_size < 0 || ros_v.limiters.size() > FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] limiters_size=" << ros_v.limiters_size 
                << " ros_v.limiters.size()=" << ros_v.limiters.size()
                << " not in range FUSION_PARKING_SLOT_LIMITER_MAX_NUM=" << FUSION_PARKING_SLOT_LIMITER_MAX_NUM 
                << std::endl;
    }
    if (ros_v.limiters.size() > FUSION_PARKING_SLOT_LIMITER_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_PARKING_SLOT_LIMITER_MAX_NUM; i1++) {
        convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.limiters.size(); i1++) {
        convert(struct_v.limiters[i1], ros_v.limiters[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::HmiRpaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rpa_available, ros_v.rpa_available, type);
  convert(struct_v.rpa_active_resp, ros_v.rpa_active_resp, type);
  convert(struct_v.rpa_sub_func_sts, ros_v.rpa_sub_func_sts, type);
  convert(struct_v.rpa_start_button_sts, ros_v.rpa_start_button_sts, type);
  convert(struct_v.bafa_back_but_sts, ros_v.bafa_back_but_sts, type);
  convert(struct_v.bafa_front_but_sts, ros_v.bafa_front_but_sts, type);
  convert(struct_v.rpa_resume_button_sts, ros_v.rpa_resume_button_sts, type);
  convert(struct_v.bafa_fun_sts, ros_v.bafa_fun_sts, type);
  convert(struct_v.rpa_quit_notify, ros_v.rpa_quit_notify, type);
  convert(struct_v.rpa_operate_notify, ros_v.rpa_operate_notify, type);
  convert(struct_v.rpa_pause_notify, ros_v.rpa_pause_notify, type);
  convert(struct_v.rpa_self_check_sts, ros_v.rpa_self_check_sts, type);
  convert(struct_v.rpa_slot_type, ros_v.rpa_slot_type, type);
  convert(struct_v.bafa_status, ros_v.bafa_status, type);
  convert(struct_v.apa_popup, ros_v.apa_popup, type);
}

template <typename T2>
void convert(iflyauto::HmiApaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.apa_mode_status, ros_v.apa_mode_status, type);
  convert(struct_v.apa_status, ros_v.apa_status, type);
  convert(struct_v.apa_slot_info_size, ros_v.apa_slot_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.apa_slot_info_size >= 0 && struct_v.apa_slot_info_size <= FUSION_PARKING_SLOT_MAX_NUM) {
      ros_v.apa_slot_info.resize(struct_v.apa_slot_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] apa_slot_info_size=" << struct_v.apa_slot_info_size 
                << " not in range FUSION_PARKING_SLOT_MAX_NUM=" << FUSION_PARKING_SLOT_MAX_NUM 
                << std::endl;
      ros_v.apa_slot_info_size = FUSION_PARKING_SLOT_MAX_NUM;
      ros_v.apa_slot_info.resize(FUSION_PARKING_SLOT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.apa_slot_info.size(); i0++) {
      convert(struct_v.apa_slot_info[i0], ros_v.apa_slot_info[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.apa_slot_info_size > FUSION_PARKING_SLOT_MAX_NUM || ros_v.apa_slot_info_size < 0 || ros_v.apa_slot_info.size() > FUSION_PARKING_SLOT_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] apa_slot_info_size=" << ros_v.apa_slot_info_size 
                << " ros_v.apa_slot_info.size()=" << ros_v.apa_slot_info.size()
                << " not in range FUSION_PARKING_SLOT_MAX_NUM=" << FUSION_PARKING_SLOT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.apa_slot_info.size() > FUSION_PARKING_SLOT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_PARKING_SLOT_MAX_NUM; i0++) {
        convert(struct_v.apa_slot_info[i0], ros_v.apa_slot_info[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.apa_slot_info.size(); i0++) {
        convert(struct_v.apa_slot_info[i0], ros_v.apa_slot_info[i0], type);
      }
    }
  }
  //
  convert(struct_v.select_slot_id, ros_v.select_slot_id, type);
  convert(struct_v.apa_notify_req, ros_v.apa_notify_req, type);
  convert(struct_v.apa_notify_remain_distance, ros_v.apa_notify_remain_distance, type);
  convert(struct_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
  convert(struct_v.apa_free_slot_info, ros_v.apa_free_slot_info, type);
  convert(struct_v.prepare_plan_state, ros_v.prepare_plan_state, type);
  convert(struct_v.planning_park_dir, ros_v.planning_park_dir, type);
  convert(struct_v.select_park_out_dir, ros_v.select_park_out_dir, type);
  convert(struct_v.select_park_in_dir, ros_v.select_park_in_dir, type);
  convert(struct_v.parking_time, ros_v.parking_time, type);
  convert(struct_v.hmi_rap_info, ros_v.hmi_rap_info, type);
  convert(struct_v.apa_active_resp, ros_v.apa_active_resp, type);
  convert(struct_v.apa_active_denied, ros_v.apa_active_denied, type);
  convert(struct_v.avm_screen_view, ros_v.avm_screen_view, type);
  convert(struct_v.apa_active_button_sts, ros_v.apa_active_button_sts, type);
  convert(struct_v.park_in_button_sts, ros_v.park_in_button_sts, type);
  convert(struct_v.park_out_button_sts, ros_v.park_out_button_sts, type);
  convert(struct_v.free_park_button_sts, ros_v.free_park_button_sts, type);
  convert(struct_v.apa_start_button_sts, ros_v.apa_start_button_sts, type);
  convert(struct_v.apa_resume_button_sts, ros_v.apa_resume_button_sts, type);
  convert(struct_v.rpa_active_button_sts, ros_v.rpa_active_button_sts, type);
  convert(struct_v.leave_car_park_button_sts, ros_v.leave_car_park_button_sts, type);
  convert(struct_v.apa_user_preference, ros_v.apa_user_preference, type);
  convert(struct_v.parking_speed_set, ros_v.parking_speed_set, type);
  convert(struct_v.parking_view_ctrl, ros_v.parking_view_ctrl, type);
  convert(struct_v.parking_arrow, ros_v.parking_arrow, type);
}

template <typename T2>
void convert(iflyauto::SpeedOffsetInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.offset_mode, ros_v.offset_mode, type);
  convert(struct_v.speed_offset_value, ros_v.speed_offset_value, type);
  convert(struct_v.speed_offset_percentage, ros_v.speed_offset_percentage, type);
}

template <typename T2>
void convert(iflyauto::HmiAccInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acc_status, ros_v.acc_status, type);
  convert(struct_v.acc_active_resp, ros_v.acc_active_resp, type);
  convert(struct_v.acc_driver_denied, ros_v.acc_driver_denied, type);
  convert(struct_v.acc_takeover_req_lv, ros_v.acc_takeover_req_lv, type);
  convert(struct_v.acc_set_headway, ros_v.acc_set_headway, type);
  convert(struct_v.acc_set_speed, ros_v.acc_set_speed, type);
  convert(struct_v.intelligent_following, ros_v.intelligent_following, type);
  convert(struct_v.acc_notify_req, ros_v.acc_notify_req, type);
  convert(struct_v.acc_notify, ros_v.acc_notify, type);
  convert(struct_v.distraction_warning, ros_v.distraction_warning, type);
  convert(struct_v.speed_offset_set, ros_v.speed_offset_set, type);
  convert(struct_v.pilot_user_preference, ros_v.pilot_user_preference, type);
  convert(struct_v.acc_spd_mode_ind, ros_v.acc_spd_mode_ind, type);
}

template <typename T2>
void convert(iflyauto::HmiLaneChange &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lc_status, ros_v.lc_status, type);
  convert(struct_v.lc_direction, ros_v.lc_direction, type);
  convert(struct_v.lc_reason, ros_v.lc_reason, type);
  convert(struct_v.obstacle_id, ros_v.obstacle_id, type);
  convert(struct_v.landing_point, ros_v.landing_point, type);
}

template <typename T2>
void convert(iflyauto::HmiIntelligentEvasion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.dodge_type, ros_v.dodge_type, type);
  convert(struct_v.object_id, ros_v.object_id, type);
}

template <typename T2>
void convert(iflyauto::HmiSccInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.scc_status, ros_v.scc_status, type);
  convert(struct_v.scc_active_resp, ros_v.scc_active_resp, type);
  convert(struct_v.scc_driver_denied, ros_v.scc_driver_denied, type);
  convert(struct_v.scc_hands_off_warning, ros_v.scc_hands_off_warning, type);
  convert(struct_v.scc_takeover_req_lv, ros_v.scc_takeover_req_lv, type);
  convert(struct_v.scc_line_detect_status, ros_v.scc_line_detect_status, type);
  convert(struct_v.intelligent_evasion, ros_v.intelligent_evasion, type);
  convert(struct_v.lane_change, ros_v.lane_change, type);
  convert(struct_v.narrow_road_tips, ros_v.narrow_road_tips, type);
  convert(struct_v.scc_notify_req, ros_v.scc_notify_req, type);
  convert(struct_v.scc_notify, ros_v.scc_notify, type);
  convert(struct_v.hands_off_detection, ros_v.hands_off_detection, type);
  convert(struct_v.traffic_light_stop_go, ros_v.traffic_light_stop_go, type);
  convert(struct_v.obstacle_bypass, ros_v.obstacle_bypass, type);
}

template <typename T2>
void convert(iflyauto::HmiAdasInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ldw_output_info, ros_v.ldw_output_info, type);
  convert(struct_v.ldp_output_info, ros_v.ldp_output_info, type);
  convert(struct_v.elk_output_info, ros_v.elk_output_info, type);
  convert(struct_v.tsr_output_info, ros_v.tsr_output_info, type);
  convert(struct_v.ihc_output_info, ros_v.ihc_output_info, type);
  convert(struct_v.ama_output_info, ros_v.ama_output_info, type);
  convert(struct_v.meb_output_info, ros_v.meb_output_info, type);
  convert(struct_v.supp_signs_size, ros_v.supp_signs_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.supp_signs_size >= 0 && struct_v.supp_signs_size <= CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      ros_v.supp_signs.resize(struct_v.supp_signs_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] supp_signs_size=" << struct_v.supp_signs_size 
                << " not in range CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM=" << CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM 
                << std::endl;
      ros_v.supp_signs_size = CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM;
      ros_v.supp_signs.resize(CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.supp_signs.size(); i0++) {
      convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.supp_signs_size > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM || ros_v.supp_signs_size < 0 || ros_v.supp_signs.size() > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] supp_signs_size=" << ros_v.supp_signs_size 
                << " ros_v.supp_signs.size()=" << ros_v.supp_signs.size()
                << " not in range CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM=" << CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.supp_signs.size() > CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM; i0++) {
        convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.supp_signs.size(); i0++) {
        convert(struct_v.supp_signs[i0], ros_v.supp_signs[i0], type);
      }
    }
  }
  //
  convert(struct_v.traffic_lights_size, ros_v.traffic_lights_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.traffic_lights_size >= 0 && struct_v.traffic_lights_size <= CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      ros_v.traffic_lights.resize(struct_v.traffic_lights_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] traffic_lights_size=" << struct_v.traffic_lights_size 
                << " not in range CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM=" << CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM 
                << std::endl;
      ros_v.traffic_lights_size = CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM;
      ros_v.traffic_lights.resize(CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.traffic_lights.size(); i1++) {
      convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.traffic_lights_size > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM || ros_v.traffic_lights_size < 0 || ros_v.traffic_lights.size() > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] traffic_lights_size=" << ros_v.traffic_lights_size 
                << " ros_v.traffic_lights.size()=" << ros_v.traffic_lights.size()
                << " not in range CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM=" << CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.traffic_lights.size() > CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM; i1++) {
        convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.traffic_lights.size(); i1++) {
        convert(struct_v.traffic_lights[i1], ros_v.traffic_lights[i1], type);
      }
    }
  }
  //
  convert(struct_v.current_lane_traffic_light, ros_v.current_lane_traffic_light, type);
}

template <typename T2>
void convert(iflyauto::HmiSensorInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.sensor_type, ros_v.sensor_type, type);
  convert(struct_v.sensor_state, ros_v.sensor_state, type);
}

template <typename T2>
void convert(iflyauto::HmiSwitchInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acc_switch_response, ros_v.acc_switch_response, type);
  convert(struct_v.lcc_switch_response, ros_v.lcc_switch_response, type);
  convert(struct_v.noa_switch_response, ros_v.noa_switch_response, type);
  convert(struct_v.apa_switch_response, ros_v.apa_switch_response, type);
  convert(struct_v.rpa_switch_response, ros_v.rpa_switch_response, type);
  convert(struct_v.hpp_switch_response, ros_v.hpp_switch_response, type);
  convert(struct_v.aeb_switch_response, ros_v.aeb_switch_response, type);
  convert(struct_v.meb_switch_response, ros_v.meb_switch_response, type);
  convert(struct_v.tsr_switch_response, ros_v.tsr_switch_response, type);
  convert(struct_v.ihc_switch_response, ros_v.ihc_switch_response, type);
  convert(struct_v.ldw_switch_response, ros_v.ldw_switch_response, type);
  convert(struct_v.ldw_level_response, ros_v.ldw_level_response, type);
  convert(struct_v.elk_switch_response, ros_v.elk_switch_response, type);
  convert(struct_v.bsd_switch_response, ros_v.bsd_switch_response, type);
  convert(struct_v.lca_switch_response, ros_v.lca_switch_response, type);
  convert(struct_v.dow_switch_response, ros_v.dow_switch_response, type);
  convert(struct_v.fcta_switch_response, ros_v.fcta_switch_response, type);
  convert(struct_v.fctb_switch_response, ros_v.fctb_switch_response, type);
  convert(struct_v.rcta_switch_response, ros_v.rcta_switch_response, type);
  convert(struct_v.rctb_switch_response, ros_v.rctb_switch_response, type);
  convert(struct_v.rcw_switch_response, ros_v.rcw_switch_response, type);
  convert(struct_v.ldp_switch_response, ros_v.ldp_switch_response, type);
  convert(struct_v.fcw_switch_response, ros_v.fcw_switch_response, type);
  convert(struct_v.rads_switch_response, ros_v.rads_switch_response, type);
  convert(struct_v.pa_switch_response, ros_v.pa_switch_response, type);
  convert(struct_v.nra_switch_response, ros_v.nra_switch_response, type);
  convert(struct_v.fcw_level_response, ros_v.fcw_level_response, type);
  convert(struct_v.amap_switch_response, ros_v.amap_switch_response, type);
  convert(struct_v.dai_switch_response, ros_v.dai_switch_response, type);
  convert(struct_v.mnp_switch_response, ros_v.mnp_switch_response, type);
  convert(struct_v.dow_secondary_alert_switch_response, ros_v.dow_secondary_alert_switch_response, type);
  convert(struct_v.blue_light_switch_response, ros_v.blue_light_switch_response, type);
  convert(struct_v.function_degrade_switch_response, ros_v.function_degrade_switch_response, type);
  convert(struct_v.speed_set_faile_rsp, ros_v.speed_set_faile_rsp, type);
  convert(struct_v.interval_set_fail_rsp, ros_v.interval_set_fail_rsp, type);
  convert(struct_v.parking_preference_faile_rsp, ros_v.parking_preference_faile_rsp, type);
}

template <typename T2>
void convert(iflyauto::Traj_Point &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.point, ros_v.point, type);
}

template <typename T2>
void convert(iflyauto::HmiHppOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.hpp_notify_req, ros_v.hpp_notify_req, type);
  convert(struct_v.hpp_status, ros_v.hpp_status, type);
  convert(struct_v.memory_parking_available, ros_v.memory_parking_available, type);
  convert(struct_v.is_first_time_using, ros_v.is_first_time_using, type);
  convert(struct_v.memory_parking_resume_available, ros_v.memory_parking_resume_available, type);
  convert(struct_v.route_learning_available, ros_v.route_learning_available, type);
  convert(struct_v.speed_bumps_count, ros_v.speed_bumps_count, type);
  convert(struct_v.learning_distance, ros_v.learning_distance, type);
  convert(struct_v.distance_to_parking_space, ros_v.distance_to_parking_space, type);
  convert(struct_v.estimated_remaining_time, ros_v.estimated_remaining_time, type);
  convert(struct_v.pedestrian_avoidance_count, ros_v.pedestrian_avoidance_count, type);
  convert(struct_v.vehicle_avoidance_count, ros_v.vehicle_avoidance_count, type);
  convert(struct_v.hpp_time_minute, ros_v.hpp_time_minute, type);
  convert(struct_v.hpp_takeover_req_lv, ros_v.hpp_takeover_req_lv, type);
  convert(struct_v.is_position_need_refresh, ros_v.is_position_need_refresh, type);
  convert(struct_v.his_traj_point_size, ros_v.his_traj_point_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.his_traj_point_size >= 0 && struct_v.his_traj_point_size <= HMI_HPP_HIS_TRAJ_POINT_MAX_NUM) {
      ros_v.his_traj_point.resize(struct_v.his_traj_point_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] his_traj_point_size=" << struct_v.his_traj_point_size 
                << " not in range HMI_HPP_HIS_TRAJ_POINT_MAX_NUM=" << HMI_HPP_HIS_TRAJ_POINT_MAX_NUM 
                << std::endl;
      ros_v.his_traj_point_size = HMI_HPP_HIS_TRAJ_POINT_MAX_NUM;
      ros_v.his_traj_point.resize(HMI_HPP_HIS_TRAJ_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.his_traj_point.size(); i0++) {
      convert(struct_v.his_traj_point[i0], ros_v.his_traj_point[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.his_traj_point_size > HMI_HPP_HIS_TRAJ_POINT_MAX_NUM || ros_v.his_traj_point_size < 0 || ros_v.his_traj_point.size() > HMI_HPP_HIS_TRAJ_POINT_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] his_traj_point_size=" << ros_v.his_traj_point_size 
                << " ros_v.his_traj_point.size()=" << ros_v.his_traj_point.size()
                << " not in range HMI_HPP_HIS_TRAJ_POINT_MAX_NUM=" << HMI_HPP_HIS_TRAJ_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.his_traj_point.size() > HMI_HPP_HIS_TRAJ_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < HMI_HPP_HIS_TRAJ_POINT_MAX_NUM; i0++) {
        convert(struct_v.his_traj_point[i0], ros_v.his_traj_point[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.his_traj_point.size(); i0++) {
        convert(struct_v.his_traj_point[i0], ros_v.his_traj_point[i0], type);
      }
    }
  }
  //
  convert(struct_v.start_point, ros_v.start_point, type);
  convert(struct_v.end_point, ros_v.end_point, type);
  convert(struct_v.hpp_active_resp, ros_v.hpp_active_resp, type);
  convert(struct_v.hpp_active_denied, ros_v.hpp_active_denied, type);
  convert(struct_v.hpa_active_button_sts, ros_v.hpa_active_button_sts, type);
  convert(struct_v.parking_space_count, ros_v.parking_space_count, type);
  convert(struct_v.route_saving_process, ros_v.route_saving_process, type);
  convert(struct_v.guidance_step, ros_v.guidance_step, type);
  convert(struct_v.cruise_distance, ros_v.cruise_distance, type);
  convert(struct_v.gear_p_request, ros_v.gear_p_request, type);
  convert(struct_v.hpp_button_display, ros_v.hpp_button_display, type);
  convert(struct_v.hpp_Remind, ros_v.hpp_Remind, type);
  convert(struct_v.notify_type_recommendation, ros_v.notify_type_recommendation, type);
}

template <typename T2>
void convert(iflyauto::HmiLaneInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.relative_id, ros_v.relative_id, type);
  convert(struct_v.lane_types_size, ros_v.lane_types_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_types_size >= 0 && struct_v.lane_types_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.lane_types.resize(struct_v.lane_types_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_types_size=" << struct_v.lane_types_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.lane_types_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.lane_types.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
      convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_types_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.lane_types_size < 0 || ros_v.lane_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_types_size=" << ros_v.lane_types_size 
                << " ros_v.lane_types.size()=" << ros_v.lane_types.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    }
  }
  //
  convert(struct_v.merge_split_points_size, ros_v.merge_split_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.merge_split_points_size >= 0 && struct_v.merge_split_points_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.merge_split_points.resize(struct_v.merge_split_points_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] merge_split_points_size=" << struct_v.merge_split_points_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.merge_split_points_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.merge_split_points.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.merge_split_points.size(); i1++) {
      convert(struct_v.merge_split_points[i1], ros_v.merge_split_points[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.merge_split_points_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.merge_split_points_size < 0 || ros_v.merge_split_points.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] merge_split_points_size=" << ros_v.merge_split_points_size 
                << " ros_v.merge_split_points.size()=" << ros_v.merge_split_points.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.merge_split_points.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i1++) {
        convert(struct_v.merge_split_points[i1], ros_v.merge_split_points[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.merge_split_points.size(); i1++) {
        convert(struct_v.merge_split_points[i1], ros_v.merge_split_points[i1], type);
      }
    }
  }
  //
  convert(struct_v.speed_limit, ros_v.speed_limit, type);
}

template <typename T2>
void convert(iflyauto::HmiStopLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.point.size(); i0++) {
	  convert(struct_v.point[i0], ros_v.point[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::HmiLaneGroundMarking &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.ground_marking_points_set.size(); i0++) {
	  convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
  }
  convert(struct_v.orientation_angle, ros_v.orientation_angle, type);
  convert(struct_v.turn_type, ros_v.turn_type, type);
}

template <typename T2>
void convert(iflyauto::HmiRadsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.rads_status, ros_v.rads_status, type);
  convert(struct_v.rads_notify_req, ros_v.rads_notify_req, type);
  convert(struct_v.rads_remain_distance, ros_v.rads_remain_distance, type);
  convert(struct_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
  convert(struct_v.rads_active_resp, ros_v.rads_active_resp, type);
  convert(struct_v.rads_active_denied, ros_v.rads_active_denied, type);
  convert(struct_v.path_point_size, ros_v.path_point_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.path_point_size >= 0 && struct_v.path_point_size <= RADS_MAP_POINT_MAX_NUM) {
      ros_v.path_point.resize(struct_v.path_point_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] path_point_size=" << struct_v.path_point_size 
                << " not in range RADS_MAP_POINT_MAX_NUM=" << RADS_MAP_POINT_MAX_NUM 
                << std::endl;
      ros_v.path_point_size = RADS_MAP_POINT_MAX_NUM;
      ros_v.path_point.resize(RADS_MAP_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.path_point.size(); i0++) {
      convert(struct_v.path_point[i0], ros_v.path_point[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.path_point_size > RADS_MAP_POINT_MAX_NUM || ros_v.path_point_size < 0 || ros_v.path_point.size() > RADS_MAP_POINT_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] path_point_size=" << ros_v.path_point_size 
                << " ros_v.path_point.size()=" << ros_v.path_point.size()
                << " not in range RADS_MAP_POINT_MAX_NUM=" << RADS_MAP_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.path_point.size() > RADS_MAP_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < RADS_MAP_POINT_MAX_NUM; i0++) {
        convert(struct_v.path_point[i0], ros_v.path_point[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.path_point.size(); i0++) {
        convert(struct_v.path_point[i0], ros_v.path_point[i0], type);
      }
    }
  }
  //
  convert(struct_v.rads_active_button_sts, ros_v.rads_active_button_sts, type);
  convert(struct_v.rads_start_button_sts, ros_v.rads_start_button_sts, type);
  convert(struct_v.rads_continue_button_sts, ros_v.rads_continue_button_sts, type);
}

template <typename T2>
void convert(iflyauto::HmiPaInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pa_status, ros_v.pa_status, type);
  convert(struct_v.pa_notify, ros_v.pa_notify, type);
  convert(struct_v.pa_active_resp, ros_v.pa_active_resp, type);
  convert(struct_v.pa_active_denied, ros_v.pa_active_denied, type);
  convert(struct_v.pa_remain_distance, ros_v.pa_remain_distance, type);
  convert(struct_v.remain_distance_percentage, ros_v.remain_distance_percentage, type);
  convert(struct_v.pa_direction, ros_v.pa_direction, type);
  for (size_t i0 = 0; i0 < ros_v.available_direction.size(); i0++) {
	  convert(struct_v.available_direction[i0], ros_v.available_direction[i0], type);
  }
  convert(struct_v.ftba_active_button_sts, ros_v.ftba_active_button_sts, type);
  convert(struct_v.pa_continue_button_sts, ros_v.pa_continue_button_sts, type);
  convert(struct_v.pa_start_button_sts, ros_v.pa_start_button_sts, type);
}

template <typename T2>
void convert(iflyauto::HmiNraInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.nra_status, ros_v.nra_status, type);
  convert(struct_v.nra_notify, ros_v.nra_notify, type);
  convert(struct_v.nra_active_resp, ros_v.nra_active_resp, type);
  convert(struct_v.nra_active_denied, ros_v.nra_active_denied, type);
  convert(struct_v.nra_distance, ros_v.nra_distance, type);
  convert(struct_v.nra_active_button_sts, ros_v.nra_active_button_sts, type);
  convert(struct_v.nra_start_button_sts, ros_v.nra_start_button_sts, type);
  convert(struct_v.nra_continue_button_sts, ros_v.nra_continue_button_sts, type);
}

template <typename T2>
void convert(iflyauto::BlueLightSignalCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.blue_light_state, ros_v.blue_light_state, type);
}

template <typename T2>
void convert(iflyauto::HmiSocOuter &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.hmi_common, ros_v.hmi_common, type);
  convert(struct_v.hmi_line_topo_size, ros_v.hmi_line_topo_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_line_topo_size >= 0 && struct_v.hmi_line_topo_size <= HMI_TOPO_LINE_MAX_NUM) {
      ros_v.hmi_line_topo.resize(struct_v.hmi_line_topo_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_line_topo_size=" << struct_v.hmi_line_topo_size 
                << " not in range HMI_TOPO_LINE_MAX_NUM=" << HMI_TOPO_LINE_MAX_NUM 
                << std::endl;
      ros_v.hmi_line_topo_size = HMI_TOPO_LINE_MAX_NUM;
      ros_v.hmi_line_topo.resize(HMI_TOPO_LINE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.hmi_line_topo.size(); i0++) {
      convert(struct_v.hmi_line_topo[i0], ros_v.hmi_line_topo[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_line_topo_size > HMI_TOPO_LINE_MAX_NUM || ros_v.hmi_line_topo_size < 0 || ros_v.hmi_line_topo.size() > HMI_TOPO_LINE_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_line_topo_size=" << ros_v.hmi_line_topo_size 
                << " ros_v.hmi_line_topo.size()=" << ros_v.hmi_line_topo.size()
                << " not in range HMI_TOPO_LINE_MAX_NUM=" << HMI_TOPO_LINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_line_topo.size() > HMI_TOPO_LINE_MAX_NUM) {
      for (size_t i0 = 0; i0 < HMI_TOPO_LINE_MAX_NUM; i0++) {
        convert(struct_v.hmi_line_topo[i0], ros_v.hmi_line_topo[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.hmi_line_topo.size(); i0++) {
        convert(struct_v.hmi_line_topo[i0], ros_v.hmi_line_topo[i0], type);
      }
    }
  }
  //
  convert(struct_v.hmi_line_size, ros_v.hmi_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_line_size >= 0 && struct_v.hmi_line_size <= CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      ros_v.hmi_line.resize(struct_v.hmi_line_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_line_size=" << struct_v.hmi_line_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM 
                << std::endl;
      ros_v.hmi_line_size = CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM;
      ros_v.hmi_line.resize(CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.hmi_line.size(); i1++) {
      convert(struct_v.hmi_line[i1], ros_v.hmi_line[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_line_size > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM || ros_v.hmi_line_size < 0 || ros_v.hmi_line.size() > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_line_size=" << ros_v.hmi_line_size 
                << " ros_v.hmi_line.size()=" << ros_v.hmi_line.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_line.size() > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM; i1++) {
        convert(struct_v.hmi_line[i1], ros_v.hmi_line[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.hmi_line.size(); i1++) {
        convert(struct_v.hmi_line[i1], ros_v.hmi_line[i1], type);
      }
    }
  }
  //
  convert(struct_v.hmi_stop_line_size, ros_v.hmi_stop_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_stop_line_size >= 0 && struct_v.hmi_stop_line_size <= CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      ros_v.hmi_stop_line.resize(struct_v.hmi_stop_line_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_stop_line_size=" << struct_v.hmi_stop_line_size 
                << " not in range CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM 
                << std::endl;
      ros_v.hmi_stop_line_size = CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM;
      ros_v.hmi_stop_line.resize(CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.hmi_stop_line.size(); i2++) {
      convert(struct_v.hmi_stop_line[i2], ros_v.hmi_stop_line[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_stop_line_size > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM || ros_v.hmi_stop_line_size < 0 || ros_v.hmi_stop_line.size() > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_stop_line_size=" << ros_v.hmi_stop_line_size 
                << " ros_v.hmi_stop_line.size()=" << ros_v.hmi_stop_line.size()
                << " not in range CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_stop_line.size() > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM; i2++) {
        convert(struct_v.hmi_stop_line[i2], ros_v.hmi_stop_line[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.hmi_stop_line.size(); i2++) {
        convert(struct_v.hmi_stop_line[i2], ros_v.hmi_stop_line[i2], type);
      }
    }
  }
  //
  convert(struct_v.hmi_inhibit_line_size, ros_v.hmi_inhibit_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_inhibit_line_size >= 0 && struct_v.hmi_inhibit_line_size <= CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      ros_v.hmi_inhibit_line.resize(struct_v.hmi_inhibit_line_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_inhibit_line_size=" << struct_v.hmi_inhibit_line_size 
                << " not in range CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM 
                << std::endl;
      ros_v.hmi_inhibit_line_size = CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM;
      ros_v.hmi_inhibit_line.resize(CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.hmi_inhibit_line.size(); i3++) {
      convert(struct_v.hmi_inhibit_line[i3], ros_v.hmi_inhibit_line[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_inhibit_line_size > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM || ros_v.hmi_inhibit_line_size < 0 || ros_v.hmi_inhibit_line.size() > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_inhibit_line_size=" << ros_v.hmi_inhibit_line_size 
                << " ros_v.hmi_inhibit_line.size()=" << ros_v.hmi_inhibit_line.size()
                << " not in range CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_inhibit_line.size() > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      for (size_t i3 = 0; i3 < CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM; i3++) {
        convert(struct_v.hmi_inhibit_line[i3], ros_v.hmi_inhibit_line[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.hmi_inhibit_line.size(); i3++) {
        convert(struct_v.hmi_inhibit_line[i3], ros_v.hmi_inhibit_line[i3], type);
      }
    }
  }
  //
  convert(struct_v.hmi_lane_ground_marking_size, ros_v.hmi_lane_ground_marking_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_lane_ground_marking_size >= 0 && struct_v.hmi_lane_ground_marking_size <= CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      ros_v.hmi_lane_ground_marking.resize(struct_v.hmi_lane_ground_marking_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_lane_ground_marking_size=" << struct_v.hmi_lane_ground_marking_size 
                << " not in range CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM=" << CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM 
                << std::endl;
      ros_v.hmi_lane_ground_marking_size = CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM;
      ros_v.hmi_lane_ground_marking.resize(CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM);
    }
    for (size_t i4 = 0; i4 < ros_v.hmi_lane_ground_marking.size(); i4++) {
      convert(struct_v.hmi_lane_ground_marking[i4], ros_v.hmi_lane_ground_marking[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_lane_ground_marking_size > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM || ros_v.hmi_lane_ground_marking_size < 0 || ros_v.hmi_lane_ground_marking.size() > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_lane_ground_marking_size=" << ros_v.hmi_lane_ground_marking_size 
                << " ros_v.hmi_lane_ground_marking.size()=" << ros_v.hmi_lane_ground_marking.size()
                << " not in range CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM=" << CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_lane_ground_marking.size() > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      for (size_t i4 = 0; i4 < CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM; i4++) {
        convert(struct_v.hmi_lane_ground_marking[i4], ros_v.hmi_lane_ground_marking[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.hmi_lane_ground_marking.size(); i4++) {
        convert(struct_v.hmi_lane_ground_marking[i4], ros_v.hmi_lane_ground_marking[i4], type);
      }
    }
  }
  //
  convert(struct_v.hmi_lane_info_size, ros_v.hmi_lane_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_lane_info_size >= 0 && struct_v.hmi_lane_info_size <= CAMERA_PERCEPTION_LANE_MAX_NUM) {
      ros_v.hmi_lane_info.resize(struct_v.hmi_lane_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_lane_info_size=" << struct_v.hmi_lane_info_size 
                << " not in range CAMERA_PERCEPTION_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_MAX_NUM 
                << std::endl;
      ros_v.hmi_lane_info_size = CAMERA_PERCEPTION_LANE_MAX_NUM;
      ros_v.hmi_lane_info.resize(CAMERA_PERCEPTION_LANE_MAX_NUM);
    }
    for (size_t i5 = 0; i5 < ros_v.hmi_lane_info.size(); i5++) {
      convert(struct_v.hmi_lane_info[i5], ros_v.hmi_lane_info[i5], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_lane_info_size > CAMERA_PERCEPTION_LANE_MAX_NUM || ros_v.hmi_lane_info_size < 0 || ros_v.hmi_lane_info.size() > CAMERA_PERCEPTION_LANE_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_lane_info_size=" << ros_v.hmi_lane_info_size 
                << " ros_v.hmi_lane_info.size()=" << ros_v.hmi_lane_info.size()
                << " not in range CAMERA_PERCEPTION_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_lane_info.size() > CAMERA_PERCEPTION_LANE_MAX_NUM) {
      for (size_t i5 = 0; i5 < CAMERA_PERCEPTION_LANE_MAX_NUM; i5++) {
        convert(struct_v.hmi_lane_info[i5], ros_v.hmi_lane_info[i5], type);
      }
    } else {
      for (size_t i5 = 0; i5 < ros_v.hmi_lane_info.size(); i5++) {
        convert(struct_v.hmi_lane_info[i5], ros_v.hmi_lane_info[i5], type);
      }
    }
  }
  //
  convert(struct_v.hmi_obj_info_size, ros_v.hmi_obj_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.hmi_obj_info_size >= 0 && struct_v.hmi_obj_info_size <= FUSION_OBJECT_MAX_NUM) {
      ros_v.hmi_obj_info.resize(struct_v.hmi_obj_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] hmi_obj_info_size=" << struct_v.hmi_obj_info_size 
                << " not in range FUSION_OBJECT_MAX_NUM=" << FUSION_OBJECT_MAX_NUM 
                << std::endl;
      ros_v.hmi_obj_info_size = FUSION_OBJECT_MAX_NUM;
      ros_v.hmi_obj_info.resize(FUSION_OBJECT_MAX_NUM);
    }
    for (size_t i6 = 0; i6 < ros_v.hmi_obj_info.size(); i6++) {
      convert(struct_v.hmi_obj_info[i6], ros_v.hmi_obj_info[i6], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.hmi_obj_info_size > FUSION_OBJECT_MAX_NUM || ros_v.hmi_obj_info_size < 0 || ros_v.hmi_obj_info.size() > FUSION_OBJECT_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] hmi_obj_info_size=" << ros_v.hmi_obj_info_size 
                << " ros_v.hmi_obj_info.size()=" << ros_v.hmi_obj_info.size()
                << " not in range FUSION_OBJECT_MAX_NUM=" << FUSION_OBJECT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.hmi_obj_info.size() > FUSION_OBJECT_MAX_NUM) {
      for (size_t i6 = 0; i6 < FUSION_OBJECT_MAX_NUM; i6++) {
        convert(struct_v.hmi_obj_info[i6], ros_v.hmi_obj_info[i6], type);
      }
    } else {
      for (size_t i6 = 0; i6 < ros_v.hmi_obj_info.size(); i6++) {
        convert(struct_v.hmi_obj_info[i6], ros_v.hmi_obj_info[i6], type);
      }
    }
  }
  //
  convert(struct_v.cipv_track_id, ros_v.cipv_track_id, type);
  convert(struct_v.cutin_track_id, ros_v.cutin_track_id, type);
  convert(struct_v.hmi_apa_info, ros_v.hmi_apa_info, type);
  convert(struct_v.hmi_acc_info, ros_v.hmi_acc_info, type);
  convert(struct_v.hmi_scc_info, ros_v.hmi_scc_info, type);
  convert(struct_v.hmi_noa_info, ros_v.hmi_noa_info, type);
  convert(struct_v.hmi_adas_info, ros_v.hmi_adas_info, type);
  convert(struct_v.calib_info, ros_v.calib_info, type);
  convert(struct_v.sensor_info_size, ros_v.sensor_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.sensor_info_size >= 0 && struct_v.sensor_info_size <= HMI_SENSOR_INFO_MAX_NUM) {
      ros_v.sensor_info.resize(struct_v.sensor_info_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] sensor_info_size=" << struct_v.sensor_info_size 
                << " not in range HMI_SENSOR_INFO_MAX_NUM=" << HMI_SENSOR_INFO_MAX_NUM 
                << std::endl;
      ros_v.sensor_info_size = HMI_SENSOR_INFO_MAX_NUM;
      ros_v.sensor_info.resize(HMI_SENSOR_INFO_MAX_NUM);
    }
    for (size_t i7 = 0; i7 < ros_v.sensor_info.size(); i7++) {
      convert(struct_v.sensor_info[i7], ros_v.sensor_info[i7], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.sensor_info_size > HMI_SENSOR_INFO_MAX_NUM || ros_v.sensor_info_size < 0 || ros_v.sensor_info.size() > HMI_SENSOR_INFO_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] sensor_info_size=" << ros_v.sensor_info_size 
                << " ros_v.sensor_info.size()=" << ros_v.sensor_info.size()
                << " not in range HMI_SENSOR_INFO_MAX_NUM=" << HMI_SENSOR_INFO_MAX_NUM 
                << std::endl;
    }
    if (ros_v.sensor_info.size() > HMI_SENSOR_INFO_MAX_NUM) {
      for (size_t i7 = 0; i7 < HMI_SENSOR_INFO_MAX_NUM; i7++) {
        convert(struct_v.sensor_info[i7], ros_v.sensor_info[i7], type);
      }
    } else {
      for (size_t i7 = 0; i7 < ros_v.sensor_info.size(); i7++) {
        convert(struct_v.sensor_info[i7], ros_v.sensor_info[i7], type);
      }
    }
  }
  //
  convert(struct_v.hmi_switch_info, ros_v.hmi_switch_info, type);
  convert(struct_v.hmi_hpp_output, ros_v.hmi_hpp_output, type);
  convert(struct_v.ehp_output, ros_v.ehp_output, type);
  convert(struct_v.running_mode, ros_v.running_mode, type);
  convert(struct_v.ground_lines_size, ros_v.ground_lines_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ground_lines_size >= 0 && struct_v.ground_lines_size <= CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      ros_v.ground_lines.resize(struct_v.ground_lines_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ground_lines_size=" << struct_v.ground_lines_size 
                << " not in range CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM 
                << std::endl;
      ros_v.ground_lines_size = CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM;
      ros_v.ground_lines.resize(CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM);
    }
    for (size_t i8 = 0; i8 < ros_v.ground_lines.size(); i8++) {
      convert(struct_v.ground_lines[i8], ros_v.ground_lines[i8], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ground_lines_size > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM || ros_v.ground_lines_size < 0 || ros_v.ground_lines.size() > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ground_lines_size=" << ros_v.ground_lines_size 
                << " ros_v.ground_lines.size()=" << ros_v.ground_lines.size()
                << " not in range CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM=" << CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM 
                << std::endl;
    }
    if (ros_v.ground_lines.size() > CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM) {
      for (size_t i8 = 0; i8 < CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM; i8++) {
        convert(struct_v.ground_lines[i8], ros_v.ground_lines[i8], type);
      }
    } else {
      for (size_t i8 = 0; i8 < ros_v.ground_lines.size(); i8++) {
        convert(struct_v.ground_lines[i8], ros_v.ground_lines[i8], type);
      }
    }
  }
  //
  convert(struct_v.decelers_size, ros_v.decelers_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.decelers_size >= 0 && struct_v.decelers_size <= CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      ros_v.decelers.resize(struct_v.decelers_size);
    } else {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << " [convert][TO_ROS] decelers_size=" << struct_v.decelers_size 
                << " not in range CAMERA_PERCEPTION_DECELERS_MAX_NUM=" << CAMERA_PERCEPTION_DECELERS_MAX_NUM 
                << std::endl;
      ros_v.decelers_size = CAMERA_PERCEPTION_DECELERS_MAX_NUM;
      ros_v.decelers.resize(CAMERA_PERCEPTION_DECELERS_MAX_NUM);
    }
    for (size_t i9 = 0; i9 < ros_v.decelers.size(); i9++) {
      convert(struct_v.decelers[i9], ros_v.decelers[i9], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.decelers_size > CAMERA_PERCEPTION_DECELERS_MAX_NUM || ros_v.decelers_size < 0 || ros_v.decelers.size() > CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      std::cout << "convert/hmi_soc_outer_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] decelers_size=" << ros_v.decelers_size 
                << " ros_v.decelers.size()=" << ros_v.decelers.size()
                << " not in range CAMERA_PERCEPTION_DECELERS_MAX_NUM=" << CAMERA_PERCEPTION_DECELERS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.decelers.size() > CAMERA_PERCEPTION_DECELERS_MAX_NUM) {
      for (size_t i9 = 0; i9 < CAMERA_PERCEPTION_DECELERS_MAX_NUM; i9++) {
        convert(struct_v.decelers[i9], ros_v.decelers[i9], type);
      }
    } else {
      for (size_t i9 = 0; i9 < ros_v.decelers.size(); i9++) {
        convert(struct_v.decelers[i9], ros_v.decelers[i9], type);
      }
    }
  }
  //
  convert(struct_v.hmi_rads_info, ros_v.hmi_rads_info, type);
  convert(struct_v.hmi_pa_info, ros_v.hmi_pa_info, type);
  convert(struct_v.hmi_nra_info, ros_v.hmi_nra_info, type);
  convert(struct_v.hmi_sr_info, ros_v.hmi_sr_info, type);
  convert(struct_v.door_lock_req, ros_v.door_lock_req, type);
  convert(struct_v.hmi_turn_signal_command, ros_v.hmi_turn_signal_command, type);
  convert(struct_v.hmi_light_signal_command, ros_v.hmi_light_signal_command, type);
  convert(struct_v.hmi_blue_light_signal_command, ros_v.hmi_blue_light_signal_command, type);
  convert(struct_v.hmi_horn_signal_command, ros_v.hmi_horn_signal_command, type);
  convert(struct_v.hmi_rear_view_mirror_signal_command, ros_v.hmi_rear_view_mirror_signal_command, type);
  for (size_t i10 = 0; i10 < ros_v.hmi_free_space_map.size(); i10++) {
	  convert(struct_v.hmi_free_space_map[i10], ros_v.hmi_free_space_map[i10], type);
  }
  convert(struct_v.hmi_parking_voice, ros_v.hmi_parking_voice, type);
}

