#pragma once

#include "../behavior_planners/hpp_switch_to_parking_decider/hpp_switch_info.h"

namespace planning {
struct ParkingSwitchDeciderOutput {
  HppParkingSwitchInfo parking_switch_info;
  void Clear() { parking_switch_info.Clear(); }
};
}  // namespace planning