#pragma once
#include <string>
#include <unordered_map>

#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "task_basic_types.h"
namespace planning {

struct CrossingAgentDeciderOutput {
  std::unordered_map<uint32_t, bool> is_crossing_map;
};

}  // namespace planning