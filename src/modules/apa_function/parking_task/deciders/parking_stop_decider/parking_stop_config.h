#pragma once

namespace planning {
namespace apa_planner {

struct ParkStopConfig {
  // todo: buffer should be different inside slot and outside slot;
  // buffer should be different for different type obstacle;
  double lat_buffer_to_dynamic_agent;
  double lon_buffer_to_dynamic_agent;
  double lat_buffer_to_static_agent;
  double lon_buffer_to_static_agent;

  double extra_check_dist;

  double min_lon_buffer;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning