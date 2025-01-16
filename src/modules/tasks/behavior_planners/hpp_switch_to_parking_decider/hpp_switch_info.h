#pragma once

#include <climits>
namespace planning {

enum class HppTrajStitchStatus {
  NONE = 0,
  SUCCESS = 0,
  FAIL = 0,
};

struct HppParkingSwitchInfo {
  double dist_to_memory_slot;

  // memory slot has a parking path.
  bool is_memory_slot_occupied;
  bool is_memory_slot_allowed_to_park;
  // FunctionalState_HPP_CRUISE_SEARCHING, found parking slot
  bool has_parking_slot_in_hpp_searching;
  // ego can park in the seletec slot.
  bool is_selected_slot_allowed_to_park;
  HppTrajStitchStatus traj_status;

  void Clear() {
    dist_to_memory_slot = NL_NMAX;
    is_memory_slot_occupied = false;
    is_memory_slot_allowed_to_park = false;
    has_parking_slot_in_hpp_searching = false;
    is_selected_slot_allowed_to_park = false;
    return;
  }
};

}  // namespace planning