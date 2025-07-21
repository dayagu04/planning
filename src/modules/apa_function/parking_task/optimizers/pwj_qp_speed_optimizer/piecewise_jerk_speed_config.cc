#include "piecewise_jerk_speed_config.h"
#include "apa_param_config.h"

namespace planning {
namespace apa_planner {
void PiecewiseJerkSpeedQPConfig::Init() {
  acc_weight = 10.0;
  // jerk_weight = 1e-4;
  jerk_weight = 0.001;
  ref_v_weight = 5.0;
  ref_s_weight = 1.0;

  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;
  enable_qp_by_path_length = speed_config.min_path_dist_for_speed_optimizer;
  max_cruise_speed = speed_config.default_cruise_speed;

  optimizer_time_limit = speed_config.optimizer_time_limit;
  time_resolution = 0.1;
  time_horizon = 6.0;

  return;
}
}  // namespace apa_planner
}  // namespace planning