#pragma once

namespace planning {
namespace apa_planner {
struct PiecewiseJerkSpeedQPConfig {
  double delta_time;
  double acc_weight;
  double jerk_weight;
  double ref_v_weight;
  double ref_s_weight;

  // If path is short, qp is not valid because of discret time nodes.
  double enable_qp_by_path_length;

  void Init();
};
}  // namespace apa_planner
}  // namespace planning