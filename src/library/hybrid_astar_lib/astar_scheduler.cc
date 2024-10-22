#include "astar_scheduler.h"

namespace planning {

void AstarScheduler::Process(const ParkSpaceType slot_type,
                             const ParkingTask park_task,
                             const PlanningReason plan_reason) {
  if (slot_type != ParkSpaceType::vertical) {
    return;
  }

  if (park_task != ParkingTask::parking_in) {
    return;
  }

  if (plan_reason != PlanningReason::GEOMETRY_CURVE_FAIL) {
    return;
  }
  return;
}

void AstarScheduler::clear() {
  slot_type_ = ParkSpaceType::none;
  park_task_ = ParkingTask::none;
  plan_reason_ = PlanningReason::NONE;
  astar_search_state_ = AstarSearchState::NONE;

  need_astar_search_ = false;

  return;
}

const bool AstarScheduler::IsNeedAstarSearch() const {
  return need_astar_search_;
}

AstarScheduler* AstarScheduler::GetAstarScheduler() {
  static AstarScheduler astar_scheduler_;

  return &astar_scheduler_;
}

}  // namespace planning