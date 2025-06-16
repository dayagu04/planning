#pragma once

namespace planning {
namespace apa_planner {

struct ParkSpeedLimitConfig {
  double default_cruise_speed;

  // todo: speed limit should designed to linear shape, not ladder shape.
  // parameter: if gap is big, ego need speed down.
  double kappa_switch_in_path_point;
  double speed_limit_by_kappa_switch;

  // kappa speed limit related
  double kappa_thresh;
  double speed_limit_by_kappa;

  // obs dist speed limit, use first order function. y = k * x + b;
  // distance range[0.2, 0.5];
  double obs_dist_upper;
  double obs_dist_lower;
  double speed_limit_lower_by_obs;
  double first_order_param_by_obs;
  double zero_order_param_by_obs;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning