#pragma once

namespace planning {
namespace apa_planner {

struct ParkStopConfig {
  double dynamic_agent_lat_buffer;
  double dynamic_agent_lon_buffer;
  double static_agent_lat_buffer;
  double static_agent_lon_buffer;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning