#pragma once

#include <unordered_map>

#include "task_basic_types.h"

namespace planning {

struct LateralObstacleHistoryInfo {
  bool lane_borrow = false;
  bool can_not_avoid = false;
  double ncar_count = 0;
  bool ncar_count_in = false;
  bool is_avd_car = false;
  double close_time = 0;
  double last_recv_time = 0;
  bool front_car = false;
  bool side_car = false;
  bool rear_car = false;
};

struct LateralObstacleDeciderOutput {
  std::unordered_map<uint32_t, LatObstacleDecisionType> lat_obstacle_decision;
  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      lateral_obstacle_history_info;
};

}  // namespace planning