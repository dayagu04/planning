#include "astar_scheduler.h"
#include "log_glog.h"

namespace planning {

void AstarScheduler::Process(const uint8_t slot_type, const uint8_t park_task,
                             const uint8_t plan_reason,
                             const uint8_t planning_status,
                             const uint8_t planner_type) {
  switch (slot_type) {
    case 0:
      slot_type_ = ParkSpaceType::VERTICAL;
      break;
    case 1:
      slot_type_ = ParkSpaceType::PARALLEL;
      break;
    case 2:
      slot_type_ = ParkSpaceType::SLANTING;
      break;
    default:
      slot_type_ = ParkSpaceType::NONE;
      break;
  }

  park_task_ = ParkingTask::TAIL_PARKING_IN;

  plan_reason_ = PlanningReason::NONE;
  if (plan_reason == 1 && slot_type_ == ParkSpaceType::VERTICAL &&
      planning_status == 5) {
    plan_reason_ = PlanningReason::GEOMETRY_CURVE_FAIL;
  }

  // is astar
  if (planner_type == 9) {
    return;
  }

  UpdateScheduler(slot_type_, park_task_, plan_reason_);

  return;
}

void AstarScheduler::UpdateScheduler(const ParkSpaceType slot_type,
                                     const ParkingTask park_task,
                                     const PlanningReason plan_reason) {
  need_astar_search_ = false;
  if (slot_type != ParkSpaceType::VERTICAL) {
    return;
  }

  if (park_task != ParkingTask::TAIL_PARKING_IN) {
    return;
  }

  if (plan_reason != PlanningReason::GEOMETRY_CURVE_FAIL) {
    return;
  }

  if (astar_search_state_ == AstarSearchState::FAILURE) {
    return;
  }

  need_astar_search_ = true;

  ILOG_INFO << "geometry fail, and will call astar";

  return;
}

void AstarScheduler::Clear() {
  slot_type_ = ParkSpaceType::NONE;
  park_task_ = ParkingTask::NONE;
  plan_reason_ = PlanningReason::NONE;
  astar_search_state_ = AstarSearchState::NONE;

  need_astar_search_ = false;

  return;
}

const bool AstarScheduler::IsNeedAstarSearch() const {
  return need_astar_search_;
}

void AstarScheduler::SetSchedulerState(const AstarSearchState state) {
  astar_search_state_ = state;

  if (astar_search_state_ == AstarSearchState::FAILURE) {
    need_astar_search_ = false;
  }

  return;
}

}  // namespace planning