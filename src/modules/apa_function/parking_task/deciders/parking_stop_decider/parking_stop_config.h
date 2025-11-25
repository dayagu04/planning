#pragma once

namespace planning {
namespace apa_planner {

struct DecisionBufferBySpeed {
  double lat_buffer;
  double lon_buffer;
};

struct ParkStopConfig {
  // todo: buffer should be different inside slot and outside slot;
  // buffer should be different for different type obstacle;
  double lat_buffer_to_dynamic_agent;
  double lon_buffer_to_dynamic_agent;
  double lat_buffer_to_static_agent;
  double lon_buffer_to_static_agent;

  DecisionBufferBySpeed static_obs_buffer;
  DecisionBufferBySpeed slow_speed_buffer;
  DecisionBufferBySpeed middle_speed_buffer;

  double extra_check_dist;

  double min_lon_buffer;

  bool enable_uss;
  bool enable_ground_line;
  bool enable_occ;

  // just consider moving obstacle (v > 0.1m/s). Static obstacle or low speed
  // obstacle use remain distance for now.
  bool enable_dynamic_od_veh;
  bool enable_dynamic_od_living_things;

  void Init();
};

}  // namespace apa_planner
}  // namespace planning