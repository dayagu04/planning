#include "park_speed_limit_config.h"
#include <cmath>
#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkSpeedLimitConfig::Init() {
  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  default_cruise_speed = speed_config.default_cruise_speed;

  // update path point kappa gap
  // If front wheel change 0.8 ratio, add speed limit.
  double kappa = 1.0 / apa_param.GetParam().min_turn_radius;
  kappa_switch_in_path_point = kappa;
  speed_limit_by_kappa_switch = speed_config.speed_limit_by_kappa_switch;

  // kappa limit speed
  kappa_thresh = kappa * 0.85;
  speed_limit_by_kappa = speed_config.speed_limit_by_kappa;

  // obs distance related
  // v = speed_limit_by_obs_+ a0 * dist
  obs_dist_upper = 0.5;
  obs_dist_lower = 0.2;

  speed_limit_lower_by_obs = speed_config.min_speed_limit_by_obs_dist;
  double max_speed = std::max(default_cruise_speed, speed_limit_lower_by_obs);
  first_order_param_by_obs = (max_speed - speed_limit_lower_by_obs) /
                             (obs_dist_upper - obs_dist_lower);
  zero_order_param_by_obs =
      speed_limit_lower_by_obs - first_order_param_by_obs * obs_dist_lower;

  return;
}
}  // namespace apa_planner
}  // namespace planning