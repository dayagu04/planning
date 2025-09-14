#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>  // Added for std::vector

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

struct LonRefPathDeciderOutput {
  TargetMakerInfo target_maker_info;
  DangerAgentInfo danger_agent_info;
  bool is_cross_vru_target_pre_handle = false;
};