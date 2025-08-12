#pragma once
#include <string>
#include <unordered_map>

#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "src/library/arastar_lib/hybrid_ara_data.h"
#include "task_basic_types.h"

namespace planning {

struct LateralObstacleHistoryInfo {
  bool lane_borrow = false;
  bool can_avoid = false;
  int can_avoid_count = 0;
  bool can_not_avoid = false;
  double ncar_count = 0;
  bool ncar_count_in = false;
  bool is_avd_car = false;
  bool last_is_avd_car = false;
  double close_time = 0;
  double last_recv_time = 0;
  bool front_car = false;
  bool side_car = false;
  bool rear_car = false;
  int side_2_front_count = 0;
  double overlap_ego_head_thr = 4;
  bool cut_in_or_cross = false;
  int cut_in_or_cross_count = 0;
  double front_expand_len = 0.0;
  double rear_expand_len = 0.0;
  bool maintain_avoid = false;
  bool is_behind_ego = false;
  int emergency_avoid_count = 0;
  bool emergency_avoid = false;
  bool lon_overtake_avoid = false;
  bool is_not_set = false;
};

struct FollowObstacleInfo {
  bool is_need_folow = false;
  double follow_confidence = 0;
};

enum class SearchResult { NO_SEARCH, SUCCESS, FAILED };

struct LateralObstacleDeciderOutput {
  ara_star::HybridARAStarResult hybrid_ara_result;
  std::unordered_map<uint32_t, LatObstacleDecisionType> lat_obstacle_decision;
  std::unordered_map<uint32_t, bool> is_emergency_avoid_release;
  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      lateral_obstacle_history_info;
  std::unordered_map<uint32_t, FollowObstacleInfo> follow_obstacle_info;
  std::vector<int> obstacles_id_behind_ego;
  SearchResult search_result = SearchResult::NO_SEARCH;
  bool in_intersection = false;
  bool left_borrow = true;
  bool right_borrow = true;
  std::unordered_map<uint32_t, double> obstacle_intrusion_distance_thr;
  TrajectoryPoints plan_history_traj;
  bool is_plan_history_traj_valid = false;
  TrajectoryPoints uniform_plan_history_traj;
  bool is_uniform_plan_history_traj_valid = false;

  void Clear() {
    hybrid_ara_result.Clear();
    lat_obstacle_decision.clear();
    search_result = SearchResult::NO_SEARCH;
  }
};
}  // namespace planning