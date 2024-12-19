#pragma once

#include <unordered_map>
#include "task_basic_types.h"

namespace planning {

struct LateralObstacleDeciderOutput {
  std::unordered_map<uint16_t, LatObstacleDecisionType> lat_obstacle_decision;
};

}  // namespace planning