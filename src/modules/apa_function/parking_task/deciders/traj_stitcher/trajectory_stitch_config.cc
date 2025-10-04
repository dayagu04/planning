#include "trajectory_stitch_config.h"
#include "apa_param_config.h"

namespace planning {
namespace apa_planner {

void TrajectoryStitchConfig::Init() {
  traj_min_dist = 0.2;
  enable_openloop_control = true;
  min_dist_for_open_loop_control = 0.1;

  low_vel_thresh_for_speed_smooth = 0.6;
  low_stitch_error.vel_stitch_error = 0.02;
  low_stitch_error.acc_stitch_error = 0.1;

  normal_stitch_error.vel_stitch_error = 0.1;
  normal_stitch_error.acc_stitch_error = 0.15;

  if (apa_param.GetParam().speed_config.lon_stitch_type == 0) {
    lon_stitch_type = LonStitchType::PLANNING_TRAJ;
  } else {
    lon_stitch_type = LonStitchType::VEHICLE_STATE;
  }

  return;
};

}  // namespace apa_planner
}  // namespace planning