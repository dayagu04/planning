#include "dp_speed_config.h"

#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {
void DpSpeedConfig::Init(const double path_length) {
  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  dp_cruise_speed = speed_config.default_cruise_speed;
  unit_v_for_long_path = 0.1;
  unit_s_for_long_path = 0.3;

  unit_v_for_short_path = 0.05;
  unit_s_for_short_path = 0.1;

  unit_v_for_extream_short_path = 0.03;
  unit_s_for_extream_short_path = 0.02;

  long_path_thresh = 1.5;
  extreme_short_path_thresh = speed_config.min_path_dist_for_speed_optimizer;

  // acc cost
  if (path_length > speed_config.path_thresh_for_acc_bound) {
    acceleration_limit = speed_config.long_path_acc_upper;
  } else {
    acceleration_limit = speed_config.short_path_acc_upper;
  }

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

  // update resolution
  if (path_length > long_path_thresh) {
    unit_v = unit_v_for_long_path;
    unit_s = unit_s_for_long_path;
  } else if (path_length > extreme_short_path_thresh) {
    unit_v = unit_v_for_short_path;
    unit_s = unit_s_for_short_path;
  } else {
    unit_v = unit_v_for_extream_short_path;
    unit_s = unit_s_for_extream_short_path;

    // ILOG_INFO << "need use jlt speed profile";
  }

  return;
}
}  // namespace apa_planner
}  // namespace planning