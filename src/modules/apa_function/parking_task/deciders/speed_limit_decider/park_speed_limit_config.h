#pragma once

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
namespace planning {
namespace apa_planner {

struct ParkSpeedLimitConfig {
  double default_cruise_speed;

  // todo: speed limit should designed to linear shape, not ladder shape.
  double kappa_switch_thresh;
  double speed_limit_by_kappa_switch;

  // kappa speed limit related
  double kappa_thresh;
  double speed_limit_lower_by_kappa;

  // obs dist speed limit, use first order function. y = k * x + b;
  // distance range[0.2, 0.5];
  double obs_dist_upper;
  double obs_dist_lower;
  double speed_limit_lower_by_obs;
  double first_order_param_by_obs;
  double zero_order_param_by_obs;

  void Init(const ParkingSpeedMode& park_speed_mode);
};

}  // namespace apa_planner
}  // namespace planning