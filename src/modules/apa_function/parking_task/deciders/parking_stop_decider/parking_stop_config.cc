#include "parking_stop_config.h"

#include <cmath>

#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkStopConfig::Init() {
  lat_buffer_to_dynamic_agent = 0.9;
  lon_buffer_to_dynamic_agent = 1.2;
  lat_buffer_to_static_agent = 0.06;
  lon_buffer_to_static_agent = 0.3;
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