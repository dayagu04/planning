#pragma once

#include <climits>
namespace planning {

enum class HppTrajStitchStatus {
  NONE = 0,
  SUCCESS = 0,
  FAIL = 0,
};

struct HppParkingSwitchInfo {
  // memory slot has a parking path.
  bool is_target_slot_occupied;
  bool is_target_slot_allowed_to_park;
  // FunctionalState_HPP_CRUISE_SEARCHING, found parking slot
  bool has_parking_slot_in_hpp_searching;
  // ego can park in the seletec slot.
  bool is_selected_slot_allowed_to_park;

  //For E541：stop near the routing destination before enter the parking status
  bool is_standstill_near_target_slot;
  bool is_timeout_for_target_slot_allowed_to_park;

  // HPP 巡航前方非虚拟障碍物刹停超时（用于触发 OBS_BLOCKED_TIMEOUT 上报）
  bool is_obs_blocked_timeout;

  void Clear() {
    is_target_slot_occupied = false;
    is_target_slot_allowed_to_park = false;
    has_parking_slot_in_hpp_searching = false;
    is_selected_slot_allowed_to_park = false;
    is_standstill_near_target_slot = false;
    is_timeout_for_target_slot_allowed_to_park = false;
    is_obs_blocked_timeout = false;
    return;
  }
};

}  // namespace planning