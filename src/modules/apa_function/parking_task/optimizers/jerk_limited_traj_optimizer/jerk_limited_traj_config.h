#pragma once

namespace planning {
namespace apa_planner {
struct JerkLimitedTrajConfig {
  double delta_time;

  double min_path_dist_for_veh_starting;

  double static_state_acc_upper;
  double dynamic_state_acc_upper;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning