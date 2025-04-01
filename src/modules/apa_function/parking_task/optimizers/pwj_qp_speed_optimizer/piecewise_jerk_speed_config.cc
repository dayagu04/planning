#include "piecewise_jerk_speed_config.h"

namespace planning {
namespace apa_planner {
void PiecewiseJerkSpeedQPConfig::Init() {
  // second
  delta_time = 0.06;
  acc_weight = 5.0;
  // jerk_weight = 1e-4;
  jerk_weight = 0.01;
  ref_v_weight = 10.0;
  ref_s_weight = 1.0;

  enable_qp_by_path_length = 0.6;

  return;
}
}  // namespace apa_planner
}  // namespace planning