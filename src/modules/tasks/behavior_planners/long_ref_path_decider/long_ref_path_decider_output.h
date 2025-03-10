#pragma once

#include <cstdint>

struct FollowTargetInfo {
  int32_t enable_far_slow_jlt_count;
};

struct TargetMakerInfo {
  FollowTargetInfo follow_target_info;
};

struct LonRefPathDeciderOutput {
  TargetMakerInfo target_maker_info;
};