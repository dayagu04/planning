#include "trajectory_stitch_config.h"

namespace planning {
namespace apa_planner {

void TrajectoryStitchConfig::Init() {
  traj_min_dist = 0.2;
  enable_openloop_control = true;
  min_dist_for_open_loop_control = 0.1;

  vel_stitch_error_for_closeloop = 0.25;
  acc_stitch_error_for_closeloop = 0.2;

  return;
};

}  // namespace apa_planner
}  // namespace planning