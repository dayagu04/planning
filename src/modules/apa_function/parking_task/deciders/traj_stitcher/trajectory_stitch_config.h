#pragma once

namespace planning {
namespace apa_planner {

struct LonStitchError {
  double vel_stitch_error;
  double acc_stitch_error;
};

enum class LonStitchType {
  PLANNING_TRAJ,
  VEHICLE_STATE,
};

enum class SpeedStitchType {
  PLANNING_TRAJ,
  LOCALIZATION,
  SPEED_ERROR,
};

enum class AccStitchType {
  PLANNING_TRAJ,
  LOCALIZATION,
  ACC_ERROR,
};

struct TrajectoryStitchConfig {
  // If remain traj is not long enough, use history traj to fill the remain
  // traj, to let the remain traj has a minimum distance.
  double traj_min_dist;

  // TODO:
  // We depend on open loop control to stop, and we do not publish stopping traj
  // to control if terminal distance is small. This is not safe.
  bool enable_openloop_control_for_short_traj;

  LonStitchType lon_stitch_type;
  SpeedStitchType speed_stitch_type;
  AccStitchType acc_stitch_type;

  // If traj dist is small, enter open loop control.
  double min_dist_for_open_loop_control;

  double low_vel_thresh_for_speed_smooth;
  // ego speed [0, 0.6], different speed use different error for_closeloop.
  LonStitchError low_stitch_error;
  // [0.6, +inf]
  LonStitchError normal_stitch_error;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning