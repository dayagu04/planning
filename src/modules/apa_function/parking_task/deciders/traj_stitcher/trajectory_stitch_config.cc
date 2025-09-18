#include "trajectory_stitch_config.h"

namespace planning {
namespace apa_planner {

void TrajectoryStitchConfig::Init() {
  traj_min_dist = 0.2;
  enable_openloop_control = true;
  min_dist_for_open_loop_control = 0.1;

  low_vel_thresh_for_speed_smooth = 0.5;
  low_stitch_error.vel_stitch_error = 0.05;
  low_stitch_error.acc_stitch_error = 0.1;

  normal_stitch_error.vel_stitch_error = 0.25;
  normal_stitch_error.acc_stitch_error = 0.2;

  return;
};

}  // namespace apa_planner
}  // namespace planning