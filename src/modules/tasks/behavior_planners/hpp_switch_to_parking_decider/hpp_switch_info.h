#pragma once

namespace planning {

enum class HppTrajStitchStatus {
  NONE = 0,
  SUCCESS = 0,
  FAIL = 0,
};

struct HppParkingSwitchInfo {
  double dist_to_memory_slot;

  // memory slot has a parking path.
  bool is_memory_slot_allowed_to_park;
  bool is_selected_slot_allowed_to_park;
  HppTrajStitchStatus traj_status;

  void Clear() {
    dist_to_memory_slot = 1000.0;
    is_memory_slot_allowed_to_park = false;
    is_selected_slot_allowed_to_park = false;
    return;
  }
};

}  // namespace planning