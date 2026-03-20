#pragma once

namespace planning {

struct DisplayConfig {
  // occupancy meters in per grid
  double hmap_window_resolution = 20;
  double virtual_obs_speed = 3;
  double main_window_resolution = 0.03;
  double control_window_resolution = 0.008;
  double replay_mode = false;
};

}  // namespace planning
