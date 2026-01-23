#pragma once

#include "../behavior_planners/hpp_stop_decider/hpp_stop_info.h"

namespace planning {
struct HppStopDeciderOutput {
  HppStopInfo hpp_stop_info;
  void Clear() { hpp_stop_info.Clear(); }
};
}  // namespace planning
