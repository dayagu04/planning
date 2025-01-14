#include "narrow_space_decider.h"

#include "log_glog.h"
#include "parking_scenario.h"

namespace planning {

void NarrowScenarioDecider::Process(const apa_planner::SlotType slot_type) {
  switch (slot_type) {
    case apa_planner::SlotType::PERPENDICULAR:
      slot_type_ = ParkSpaceType::VERTICAL;
      break;
    case apa_planner::SlotType::PARALLEL:
      slot_type_ = ParkSpaceType::PARALLEL;
      break;
    case apa_planner::SlotType::SLANT:
      slot_type_ = ParkSpaceType::SLANTING;
      break;
    default:
      slot_type_ = ParkSpaceType::NONE;
      break;
  }

  UpdateNarrowScenario(slot_type_);

  return;
}

void NarrowScenarioDecider::UpdateNarrowScenario(
    const ParkSpaceType slot_type) {
  is_narrow_space_ = false;
  is_need_astar_ = false;

  // 目前，只在垂直车尾入库作用.
  if (slot_type != ParkSpaceType::VERTICAL) {
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
  astar_search_state_ = AstarSearchState::NONE;

  is_narrow_space_ = false;
  is_need_astar_ = false;

  return;
}

const bool NarrowScenarioDecider::IsNarrowSpaceScenario() const {
  return is_narrow_space_;
}

const bool NarrowScenarioDecider::IsNeedAstar() const { return is_need_astar_; }

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