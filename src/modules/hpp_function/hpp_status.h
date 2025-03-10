#pragma once

namespace planning {

enum class HppTrajectoryType {
  NONE = 0,
  ON_LANE_DRIVING,
  PARKING,
};

struct HppParkingSwitchStatus {
  double dist_to_memory_slot;
  bool is_success_for_parking_planning;
  HppTrajectoryType traj_type;
};

}  // namespace planning