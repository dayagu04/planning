#pragma once

namespace planning {
namespace apa_planner {

struct TrajectoryStitchConfig {
  // If remain traj is not long enough, use history traj to fill the remain
  // traj, to let the remain traj has a minimum distance.
  double traj_min_dist;

  // TODO:
  // We depend on open loop control to stop, and we do not publish stopping traj
  // to control if terminal distance is small. This is not safe.
  bool enable_openloop_control;

  // If traj dist is small, enter open loop control.
  double min_dist_for_open_loop_control;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning