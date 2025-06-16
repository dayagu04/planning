#include "parking_stop_config.h"

#include <cmath>

#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkStopConfig::Init() {
  lat_buffer_to_dynamic_agent = 0.5;
  lon_buffer_to_dynamic_agent = 1.0;
  lat_buffer_to_static_agent = 0.06;
  lon_buffer_to_static_agent = 0.30;
  extra_check_dist = 0.3;
  min_lon_buffer = 0.25;

  return;
}
}  // namespace apa_planner
}  // namespace planning