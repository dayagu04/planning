#include "lateral_avoid_hmi_decider.h"

#include "planning_context.h"

namespace planning {

LateralAvoidHMIDecider::LateralAvoidHMIDecider(framework::Session* session){
  session_ = session;
}

void LateralAvoidHMIDecider::InitInfo() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  // 1. init ad info
  // 2. init hpp info
  // 3. init nsa info
  // 4. init rads info
  hmi_info->rads_info.aovid_id = -1;
  hmi_info->rads_info.avoid_status = iflyauto::AvoidObstacle::AVOID_NO_HIDING;
}

bool LateralAvoidHMIDecider::Execute() {
  InitInfo();
  if (session_->is_rads_scene()) {
    GenerateHMIInfoForRADS();
  } else {
    // ad info
  }
  return true;
}

bool LateralAvoidHMIDecider::GenerateHMIInfoForRADS() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();

  // bound avoid
  const auto& general_lateral_decider_output =
      session_->planning_context().general_lateral_decider_output();
  const auto& frenet_bound = general_lateral_decider_output.second_soft_bounds_frenet_point;
  const auto& bound_info = general_lateral_decider_output.second_soft_bounds_info;
  for (size_t i = 0; i < frenet_bound.size(); ++i) {
    if (frenet_bound[i].first > 0.) {
      hmi_info->rads_info.avoid_status = iflyauto::AvoidObstacle::AVOID_HIDING;
      if (i < bound_info.size()) {
        hmi_info->rads_info.aovid_id = bound_info[i].first.id;
      }
      break;
    }
    if (frenet_bound[i].second < 0.) {
      hmi_info->rads_info.avoid_status = iflyauto::AvoidObstacle::AVOID_HIDING;
      if (i < bound_info.size()) {
        hmi_info->rads_info.aovid_id = bound_info[i].second.id;
      }
      break;
    }
  }
  return true;
}

}  // namespace planning