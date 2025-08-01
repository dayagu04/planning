#pragma once

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
namespace planning {
namespace apa_planner {

struct PiecewiseJerkSpeedQPConfig {
  double acc_weight;
  double jerk_weight;
  double ref_v_weight;
  double ref_s_weight;

  double max_cruise_speed;

  // If path is short, qp is not valid because of discret time nodes.
  double enable_qp_by_path_length;

  double time_resolution;
  double time_horizon;
  double optimizer_time_limit;

  void Init(const ParkingSpeedMode& park_speed_mode);
};
}  // namespace apa_planner
}  // namespace planning