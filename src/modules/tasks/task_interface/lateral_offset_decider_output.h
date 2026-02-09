#pragma once
#include <vector>
#include <array>
#include "src/modules/tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"

namespace planning {
struct LateralOffsetDeciderOutput {
  bool is_valid = false;
  double lateral_offset = 0.0;
  bool enable_bound = false;
  std::vector<uint32_t> avoid_ids;

  // hmi
  std::array<AvoidObstacleInfo, 2> avd_obstacles;
  void Reset() {
    bool is_valid = false;
    double lateral_offset = 0.0;
    bool enable_bound = false;

    // hmi
    int avoid_id = -1;
    int avoid_direction = 0;
  }
};
} // namespace planning