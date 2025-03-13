#include "dp_speed_config.h"

#include "apa_param_config.h"

namespace planning {

void DpSpeedConfig::Init() {
  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;

  dp_cruise_speed = speed_config.default_cruise_speed;
  unit_v_for_long_path = 0.1;
  unit_s_for_long_path = 0.3;

  unit_v_for_short_path = 0.05;
  unit_s_for_short_path = 0.1;

  short_path_thresh = 1.5;
  extreme_short_path_thresh = 0.4;

  // acc cost
  acceleration_limit = speed_config.acc_upper;
  deceleration_limit = speed_config.acc_lower;
  advised_acceleration = 0.5;
  advised_deceleration = -0.2;
  acceleration_penalty = 15.0;
  deceleration_penalty = 100.0;

  // speed cost
  low_speed_limit = 0.3;
  low_speed_penalty = 15.0;
  exceed_speed_penalty = 15.0;
  ref_speed_gap_penalty = 7.0;
  stopover_penalty = 200.0;

  jerk_penalty = 1.0;

  enter_apa_speed_margin = 10.0;

  s_interpolate_step = 0.02;

  enable_dp_by_path_length = 0.6;

  return;
}

}  // namespace planning