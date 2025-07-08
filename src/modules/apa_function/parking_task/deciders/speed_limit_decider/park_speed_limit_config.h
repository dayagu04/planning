#pragma once

namespace planning {
namespace apa_planner {

struct ParkSpeedLimitConfig {
  double default_cruise_speed_;
  double min_cruise_speed_;

  // todo: speed limit should designed to linear shape, not ladder shape.
  // parameter: if gap is big, ego need speed down.
  double kappa_switch_in_path_point_;
  double speed_limit_by_kappa_switch_;

  // kappa speed limit related
  double kappa_thresh_;
  double speed_limit_by_kappa_;

  double obs_dist_thresh_;
  double speed_limit_by_obs_;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning