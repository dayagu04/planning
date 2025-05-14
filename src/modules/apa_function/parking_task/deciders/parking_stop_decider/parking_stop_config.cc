#include "parking_stop_config.h"

#include <cmath>

#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ParkStopConfig::Init() {
  dynamic_agent_lat_buffer = 0.5;
  dynamic_agent_lon_buffer = 1.0;
  static_agent_lat_buffer = 0.06;
  static_agent_lon_buffer = 0.28;

  return;
}
}  // namespace apa_planner
}  // namespace planning