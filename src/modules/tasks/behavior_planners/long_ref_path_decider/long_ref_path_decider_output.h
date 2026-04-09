#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>

struct FollowTargetInfo {
  int32_t enable_far_slow_jlt_count;
  int32_t enable_stable_jlt_count;
};

struct TargetMakerInfo {
  FollowTargetInfo follow_target_info;
};

struct DangerAgentInfo {
  std::unordered_set<int32_t> agents_id_set;
};

struct ComfortTargetUpperBoundInfo {
  double s = 0.0;
  double t = 0.0;
  double v = 0.0;
  double a = 0.0;
  int32_t agent_id = -1;
  int64_t st_boundary_id = -1;
};

struct LonRefPathDeciderOutput {
  TargetMakerInfo target_maker_info;
  DangerAgentInfo danger_agent_info;
  bool is_cross_vru_pre_handle = false;
  bool is_lat_follow = false;
  bool is_lon_cutin = false;
  bool is_lon_cipv_emergency_stop = false;
  bool is_lon_cutin_emergency_stop = false;
  std::vector<int32_t> st_overtake_agent_ids;
  int32_t parallel_overtake_agent_id;
  std::vector<ComfortTargetUpperBoundInfo> comfort_target_upper_bound_infos;
  std::vector<std::pair<double, double>> comfort_target;  // s, v
  bool is_follow_cipv = false;
};