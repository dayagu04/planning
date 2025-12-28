#pragma once
#include <string>
#include <unordered_map>

#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "src/modules/tasks/behavior_planners/lateral_obstacle_decider/ARAStar/hybrid_ara_data.h"
#include "task_basic_types.h"

namespace planning {

struct LateralObstacleHistoryInfo {
  bool lane_borrow = false;
  bool can_avoid = false; //  障碍物能否避让；  is_avd_car:true --- can_avoid一定为true;    满足计数条件后，  can_avoid为true
  int can_avoid_count = 0;
  bool can_not_avoid = false;
  double ncar_count = 0;
  bool ncar_count_in = false;
  bool is_avd_car = false;
  bool last_is_avd_car = false;
  bool has_safe_space = false;
  double close_time = 0;
  double last_recv_time = 0;
  bool front_car = false;
  bool side_car = false;
  bool rear_car = false;
  bool is_potential_avoiding_side_car = false;
  int side_2_front_count = 0;
  double overlap_ego_head_thr = 2;
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
  bool is_cross_lane = false;
  int cross_lane_count = 0;
};

struct FollowObstacleInfo {
  bool is_need_folow = false;
  double follow_confidence = 0;
};

struct TimeInterval {
  double start = 0.0;  // 决策的时间起点
  double end = 0.0;    // 决策的时间终点

  bool IsValid() const { return end >= start; }

  bool Contains(double t) const { return t >= start && t <= end; }
};

struct SpatioTemporalFollowWindow {
  LatObstacleDecisionType lateral_decision = LatObstacleDecisionType::IGNORE;
  TimeInterval passable_time_interval;
};

struct SpatioTemporalFollowObstacleInfo {
  std::vector<SpatioTemporalFollowWindow> follow_time_windows;

  const SpatioTemporalFollowWindow* QueryByTime(double t) const {
    for (const auto& window : follow_time_windows) {
      if (t < window.passable_time_interval.start) {
        break;
      }
      if (window.passable_time_interval.Contains(t)) {
        return &window;
      }
    }
    return nullptr;
  }
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
  std::unordered_map<uint32_t, SpatioTemporalFollowObstacleInfo>
      spatio_temporal_follow_obstacle_info;

  void Clear() {
    hybrid_ara_result.Clear();
    lat_obstacle_decision.clear();
    search_result = SearchResult::NO_SEARCH;
  }
};
}  // namespace planning