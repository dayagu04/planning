#pragma once

namespace planning {
struct PiecewiseJerkSpeedQPConfig {
  double delta_time;
  double acc_weight;
  double jerk_weight;
  double ref_v_weight;
  double ref_s_weight;

  void Init();
};

}  // namespace planning