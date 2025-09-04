#pragma once

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
namespace planning {
namespace apa_planner {
struct DpSpeedConfig {
  double dp_cruise_speed;
  double enter_apa_speed_margin;

  double acceleration_limit;
  double deceleration_limit;

  double advised_acceleration;
  double advised_deceleration;

  // If acc is not ideal, add a penalty
  double acceleration_penalty;
  double deceleration_penalty;

  double unit_v_for_long_path;
  double unit_s_for_long_path;

  double unit_v_for_short_path;
  double unit_s_for_short_path;

  double unit_v_for_extream_short_path;
  double unit_s_for_extream_short_path;

  double unit_v;
  double unit_s;

  double long_path_thresh;
  double extreme_short_path_thresh;

  double stopover_penalty;

  // 车速低于一定值，车辆难以控制.
  double low_speed_limit;
  double low_speed_penalty;

  double exceed_speed_penalty;
  double ref_speed_gap_penalty;

  double jerk_penalty;

  double s_interpolate_step;

  void Init(const double path_length, const ParkingSpeedMode& park_speed_mode);
};
}  // namespace apa_planner
}  // namespace planning