#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/hmi_hpp_outer_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiHppOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.func_state, ros_v.func_state, type);
  convert(struct_v.alert_msg, ros_v.alert_msg, type);
  convert(struct_v.disp_msg, ros_v.disp_msg, type);
  convert(struct_v.voice_msg, ros_v.voice_msg, type);
  convert(struct_v.memory_parking_available, ros_v.memory_parking_available, type);
  convert(struct_v.is_first_time_using, ros_v.is_first_time_using, type);
  convert(struct_v.is_start_point_reached, ros_v.is_start_point_reached, type);
  convert(struct_v.memory_parking_resume_available, ros_v.memory_parking_resume_available, type);
  convert(struct_v.speed_bumps_count, ros_v.speed_bumps_count, type);
  convert(struct_v.learning_distance, ros_v.learning_distance, type);
  convert(struct_v.distance_to_parking_space, ros_v.distance_to_parking_space, type);
  convert(struct_v.distance_remaining_to_park, ros_v.distance_remaining_to_park, type);
  convert(struct_v.shift_lever, ros_v.shift_lever, type);
  convert(struct_v.pedestrian_avoidance_count, ros_v.pedestrian_avoidance_count, type);
  convert(struct_v.vehicle_avoidance_count, ros_v.vehicle_avoidance_count, type);
  convert(struct_v.hpp_time_minute, ros_v.hpp_time_minute, type);
  convert(struct_v.apa_time_minute, ros_v.apa_time_minute, type);
  convert(struct_v.takeover_reminder, ros_v.takeover_reminder, type);
  convert(struct_v.location, ros_v.location, type);
  convert(struct_v.route_save_failure_reason, ros_v.route_save_failure_reason, type);
  convert(struct_v.is_position_need_refresh, ros_v.is_position_need_refresh, type);
}

