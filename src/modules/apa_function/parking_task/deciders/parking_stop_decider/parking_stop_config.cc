#include "parking_stop_config.h"

#include <cmath>

#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkStopConfig::Init() {
  static_obs_buffer.lat_buffer = 0.06;
  static_obs_buffer.lon_buffer = 0.3;

  slow_speed_thresh = 1.0;
  slow_speed_buffer.lat_buffer = 1.0;
  slow_speed_buffer.lon_buffer = 1.2;

  middle_speed_thresh = 6.0;
  middle_speed_buffer.lat_buffer = 1.2;
  middle_speed_buffer.lon_buffer = 2.5;

  high_speed_buffer.lat_buffer = 1.2;
  high_speed_buffer.lon_buffer = 3.5;

  extra_check_dist = 1.2;
  min_lon_buffer = 0.25;

  enable_uss = false;
  enable_ground_line = false;
  enable_occ = false;
  enable_dynamic_od_veh = true;
  enable_dynamic_od_living_things = true;

  return;
}
}  // namespace apa_planner
}  // namespace planning