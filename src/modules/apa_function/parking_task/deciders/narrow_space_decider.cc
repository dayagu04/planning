#include "narrow_space_decider.h"
#include "apa_data.h"
#include "log_glog.h"
#include "parking_scenario.h"

namespace planning {

void NarrowScenarioDecider::Process(
    const uint8_t slot_type, const uint8_t park_task,
    const uint8_t plan_reason, const uint8_t planning_status,
    const apa_planner::ParkingScenarioType scene_type) {
  switch (slot_type) {
    case Common::PARKING_SLOT_TYPE_VERTICAL:
      slot_type_ = ParkSpaceType::VERTICAL;
      break;
    case Common::PARKING_SLOT_TYPE_HORIZONTAL:
      slot_type_ = ParkSpaceType::PARALLEL;
      break;
    case Common::PARKING_SLOT_TYPE_SLANTING:
      slot_type_ = ParkSpaceType::SLANTING;
      break;
    default:
      slot_type_ = ParkSpaceType::NONE;
      break;
  }

  plan_reason_ = PlanningReason::NONE;
  if (plan_reason == apa_planner::ParkingScenario::FIRST_PLAN &&
      slot_type_ == ParkSpaceType::VERTICAL &&
      planning_status ==
          apa_planner::ParkingScenario::ParkingStatus::PARKING_FAILED) {
    plan_reason_ = PlanningReason::GEOMETRY_CURVE_FAIL;
  }

  // is astar
  if (scene_type == apa_planner::ParkingScenarioType::SCENARIO_NARROW_SPACE) {
    return;
  }

  UpdateNarrowScenario(slot_type_, plan_reason_);

  return;
}

void NarrowScenarioDecider::UpdateNarrowScenario(
    const ParkSpaceType slot_type, const PlanningReason plan_reason) {
  is_narrow_space_ = false;
  is_need_astar_ = false;
  if (slot_type != ParkSpaceType::VERTICAL) {
    return;
  }

  if (plan_reason != PlanningReason::GEOMETRY_CURVE_FAIL) {
    return;
  }

  if (astar_search_state_ == AstarSearchState::FAILURE) {
    return;
  }

  is_narrow_space_ = true;
  is_need_astar_ = true;

  ILOG_INFO << "geometry fail, and will call astar";

  return;
}

void NarrowScenarioDecider::Clear() {
  slot_type_ = ParkSpaceType::NONE;
  plan_reason_ = PlanningReason::NONE;
  astar_search_state_ = AstarSearchState::NONE;

  is_narrow_space_ = false;
  is_need_astar_ = false;

  return;
}

const bool NarrowScenarioDecider::IsNarrowSpaceScenario() const {
  return is_narrow_space_;
}

const bool NarrowScenarioDecider::IsNeedAstar() const {
  return is_need_astar_;
}

void NarrowScenarioDecider::SetAstarState(const AstarSearchState state) {
  astar_search_state_ = state;

  if (astar_search_state_ == AstarSearchState::FAILURE) {
    is_need_astar_ = false;
  } else if (astar_search_state_ == AstarSearchState::SUCCESS) {
    is_need_astar_ = true;
  }

  return;
}

}  // namespace planning