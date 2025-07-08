#pragma once

namespace planning {
namespace apa_planner {
struct JerkLimitedTrajConfig {
  double delta_time;

  double min_path_dist_for_veh_starting;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning