#pragma once

#include "base_convert.h"
#include "c/planning_plan_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::PlanMeta &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.plan_strategy_name.size(); i0++) {
	  convert(struct_v.plan_strategy_name[i0], ros_v.plan_strategy_name[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::AccelerationRange &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.min_a, ros_v.min_a, type);
  convert(struct_v.max_a, ros_v.max_a, type);
}

template <typename T2>
void convert(iflyauto::TargetReference &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.polynomial.size(); i0++) {
	  convert(struct_v.polynomial[i0], ros_v.polynomial[i0], type);
  }
  convert(struct_v.target_velocity, ros_v.target_velocity, type);
  convert(struct_v.acceleration_range_limit, ros_v.acceleration_range_limit, type);
  convert(struct_v.lateral_maneuver_gear, ros_v.lateral_maneuver_gear, type);
}

template <typename T2>
void convert(iflyauto::TrajectoryPoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.heading_yaw, ros_v.heading_yaw, type);
  convert(struct_v.curvature, ros_v.curvature, type);
  convert(struct_v.t, ros_v.t, type);
  convert(struct_v.v, ros_v.v, type);
  convert(struct_v.a, ros_v.a, type);
  convert(struct_v.distance, ros_v.distance, type);
  convert(struct_v.jerk, ros_v.jerk, type);
}

template <typename T2>
void convert(iflyauto::Trajectory &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.trajectory_type, ros_v.trajectory_type, type);
  convert(struct_v.target_reference, ros_v.target_reference, type);
  convert(struct_v.trajectory_points_size, ros_v.trajectory_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.trajectory_points_size >= 0 && struct_v.trajectory_points_size <= PLANNING_TRAJ_POINTS_MAX_NUM) {
      ros_v.trajectory_points.resize(struct_v.trajectory_points_size);
    } else {
      std::cout << "convert/planning_plan_c.h:" << __LINE__ 
                << " [convert][TO_ROS] trajectory_points_size=" << struct_v.trajectory_points_size 
                << " not in range PLANNING_TRAJ_POINTS_MAX_NUM=" << PLANNING_TRAJ_POINTS_MAX_NUM 
                << std::endl;
      ros_v.trajectory_points_size = PLANNING_TRAJ_POINTS_MAX_NUM;
      ros_v.trajectory_points.resize(PLANNING_TRAJ_POINTS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.trajectory_points.size(); i0++) {
      convert(struct_v.trajectory_points[i0], ros_v.trajectory_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.trajectory_points_size > PLANNING_TRAJ_POINTS_MAX_NUM || ros_v.trajectory_points_size < 0 || ros_v.trajectory_points.size() > PLANNING_TRAJ_POINTS_MAX_NUM) {
      std::cout << "convert/planning_plan_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] trajectory_points_size=" << ros_v.trajectory_points_size 
                << " ros_v.trajectory_points.size()=" << ros_v.trajectory_points.size()
                << " not in range PLANNING_TRAJ_POINTS_MAX_NUM=" << PLANNING_TRAJ_POINTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.trajectory_points.size() > PLANNING_TRAJ_POINTS_MAX_NUM) {
      for (size_t i0 = 0; i0 < PLANNING_TRAJ_POINTS_MAX_NUM; i0++) {
        convert(struct_v.trajectory_points[i0], ros_v.trajectory_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.trajectory_points.size(); i0++) {
        convert(struct_v.trajectory_points[i0], ros_v.trajectory_points[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::TurnSignalCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.turn_signal_value, ros_v.turn_signal_value, type);
}

template <typename T2>
void convert(iflyauto::LightSignalCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.light_signal_value, ros_v.light_signal_value, type);
}

template <typename T2>
void convert(iflyauto::HornSignalCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.horn_signal_value, ros_v.horn_signal_value, type);
}

template <typename T2>
void convert(iflyauto::GearCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.gear_command_value, ros_v.gear_command_value, type);
}

template <typename T2>
void convert(iflyauto::RearViewMirrorCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.rear_view_mirror_value, ros_v.rear_view_mirror_value, type);
}

template <typename T2>
void convert(iflyauto::OpenLoopSteeringCommand &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.jerk_factor, ros_v.jerk_factor, type);
  convert(struct_v.need_steering_wheel_stationary, ros_v.need_steering_wheel_stationary, type);
  convert(struct_v.steering_wheel_rad_limit, ros_v.steering_wheel_rad_limit, type);
}

template <typename T2>
void convert(iflyauto::ApaPlanningGearChangeStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.has_request_continue_parking, ros_v.has_request_continue_parking, type);
  convert(struct_v.remaining_gear_change_count, ros_v.remaining_gear_change_count, type);
}

template <typename T2>
void convert(iflyauto::PlanningStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.standstill, ros_v.standstill, type);
  convert(struct_v.ready_to_go, ros_v.ready_to_go, type);
  convert(struct_v.apa_planning_status, ros_v.apa_planning_status, type);
  convert(struct_v.hpp_planning_status, ros_v.hpp_planning_status, type);
  convert(struct_v.rads_planning_status, ros_v.rads_planning_status, type);
  convert(struct_v.nsa_planning_status, ros_v.nsa_planning_status, type);
  convert(struct_v.apa_planning_failed_reason, ros_v.apa_planning_failed_reason, type);
  convert(struct_v.apa_planning_gear_change_status, ros_v.apa_planning_gear_change_status, type);
}

template <typename T2>
void convert(iflyauto::PlanningRequest &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.take_over_req_level, ros_v.take_over_req_level, type);
  convert(struct_v.request_reason, ros_v.request_reason, type);
}

template <typename T2>
void convert(iflyauto::SuccessfulSlotsInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.is_narrow_slot, ros_v.is_narrow_slot, type);
}

template <typename T2>
void convert(iflyauto::PlanningOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.meta, ros_v.meta, type);
  convert(struct_v.trajectory, ros_v.trajectory, type);
  convert(struct_v.turn_signal_command, ros_v.turn_signal_command, type);
  convert(struct_v.light_signal_command, ros_v.light_signal_command, type);
  convert(struct_v.horn_signal_command, ros_v.horn_signal_command, type);
  convert(struct_v.gear_command, ros_v.gear_command, type);
  convert(struct_v.rear_view_mirror_signal_command, ros_v.rear_view_mirror_signal_command, type);
  convert(struct_v.open_loop_steering_command, ros_v.open_loop_steering_command, type);
  convert(struct_v.planning_status, ros_v.planning_status, type);
  convert(struct_v.successful_slot_info_list_size, ros_v.successful_slot_info_list_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.successful_slot_info_list_size >= 0 && struct_v.successful_slot_info_list_size <= PLANNING_PARKING_SLOT_MAX_NUM) {
      ros_v.successful_slot_info_list.resize(struct_v.successful_slot_info_list_size);
    } else {
      std::cout << "convert/planning_plan_c.h:" << __LINE__ 
                << " [convert][TO_ROS] successful_slot_info_list_size=" << struct_v.successful_slot_info_list_size 
                << " not in range PLANNING_PARKING_SLOT_MAX_NUM=" << PLANNING_PARKING_SLOT_MAX_NUM 
                << std::endl;
      ros_v.successful_slot_info_list_size = PLANNING_PARKING_SLOT_MAX_NUM;
      ros_v.successful_slot_info_list.resize(PLANNING_PARKING_SLOT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.successful_slot_info_list.size(); i0++) {
      convert(struct_v.successful_slot_info_list[i0], ros_v.successful_slot_info_list[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.successful_slot_info_list_size > PLANNING_PARKING_SLOT_MAX_NUM || ros_v.successful_slot_info_list_size < 0 || ros_v.successful_slot_info_list.size() > PLANNING_PARKING_SLOT_MAX_NUM) {
      std::cout << "convert/planning_plan_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] successful_slot_info_list_size=" << ros_v.successful_slot_info_list_size 
                << " ros_v.successful_slot_info_list.size()=" << ros_v.successful_slot_info_list.size()
                << " not in range PLANNING_PARKING_SLOT_MAX_NUM=" << PLANNING_PARKING_SLOT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.successful_slot_info_list.size() > PLANNING_PARKING_SLOT_MAX_NUM) {
      for (size_t i0 = 0; i0 < PLANNING_PARKING_SLOT_MAX_NUM; i0++) {
        convert(struct_v.successful_slot_info_list[i0], ros_v.successful_slot_info_list[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.successful_slot_info_list.size(); i0++) {
        convert(struct_v.successful_slot_info_list[i0], ros_v.successful_slot_info_list[i0], type);
      }
    }
  }
  //
  convert(struct_v.planning_request, ros_v.planning_request, type);
}

