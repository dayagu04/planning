#pragma once

namespace planning {
#define ogm_resolution (0.05)
#define ogm_grid_x_max (512)
#define ogm_grid_y_max (800)

struct OgmIndex {
  int x;
  int y;
};

struct EDTData {
  float dist[ogm_grid_x_max][ogm_grid_y_max];
};

}  // namespace planning