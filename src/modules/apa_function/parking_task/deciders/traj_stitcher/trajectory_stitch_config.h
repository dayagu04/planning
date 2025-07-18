#pragma once

namespace planning {
namespace apa_planner {

struct TrajectoryStitchConfig {
  // If remain path is not long enough, use full path to fill the remain path,
  // to let remain path has a minimum ditance.
  double traj_min_dist;

  // TODO:
  // We depend on open loop control to stop. This is not safe.
  bool enable_openloop_control;

  // If path dist is small, enter open loop control.
  double min_dist_for_open_loop_control;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning