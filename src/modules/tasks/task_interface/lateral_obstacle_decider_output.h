#pragma once
#include <string>
#include <unordered_map>

#include <utility>
#include <vector>

#include "src/library/arastar_lib/hybrid_ara_data.h"
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
  ara_star::HybridARAStarResult hybrid_ara_result;
  std::unordered_map<uint32_t, LatObstacleDecisionType> lat_obstacle_decision;
  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      lateral_obstacle_history_info;
  bool search_success = false;

  void Clear() {
    hybrid_ara_result.Clear();
    lat_obstacle_decision.clear();
    search_success = false;
  }
};
}  // namespace planning