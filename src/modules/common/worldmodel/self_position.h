#pragma once

#include <vector>

namespace planning {

enum MSDSelfPositionAvailableFlag {
  MSD_SELF_POSITION_SELF_POSITION_DATA = 1 << 0,
};

struct MSDSelfPositionData {
  bool in_map_area;
  bool in_intersection;
  bool on_ramp;
};

struct MSDSelfPosition {
  size_t available;
  MSDSelfPositionData self_position_data;
};
}  // namespace planning
