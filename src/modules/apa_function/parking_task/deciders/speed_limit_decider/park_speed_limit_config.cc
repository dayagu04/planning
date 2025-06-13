#include "park_speed_limit_config.h"
#include <cmath>
#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkSpeedLimitConfig::Init() {
  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  default_cruise_speed_ = speed_config.default_cruise_speed;
  min_cruise_speed_ = speed_config.min_cruise_speed;

  // update path point kappa gap
  // If front wheel change 0.8 ratio, add speed limit.
  double kappa = 1.0 / apa_param.GetParam().min_turn_radius;
  kappa_switch_in_path_point_ = kappa;
  speed_limit_by_kappa_switch_ = speed_config.speed_limit_by_kappa_switch;

  // kappa limit speed
  kappa_thresh_ = kappa * 0.85;
  speed_limit_by_kappa_ = speed_config.speed_limit_by_kappa;

  // obs distance related
  // v = speed_limit_by_obs_+ a0 * dist
  obs_dist_thresh_ = 0.5;
  double speed_limit_lower_by_obs_ = speed_config.min_speed_limit_by_obs_dist;
  double max_speed = std::max(0.7, speed_limit_lower_by_obs_);
  first_order_param_by_obs_ =
      (max_speed - speed_limit_lower_by_obs_) / obs_dist_thresh_;

  return;
}
}  // namespace apa_planner
}  // namespace planning