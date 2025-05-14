#include "jerk_limited_traj_config.h"
#include "apa_param_config.h"

namespace planning {
namespace apa_planner {
void JerkLimitedTrajConfig::Init() {
  // second
  delta_time = 0.05;

  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;
  min_path_dist_for_veh_starting = speed_config.min_path_dist_for_veh_starting;

  return;
}
}  // namespace apa_planner
}  // namespace planning