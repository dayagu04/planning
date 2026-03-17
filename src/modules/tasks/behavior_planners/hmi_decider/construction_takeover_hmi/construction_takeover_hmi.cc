#include "construction_takeover_hmi.h"
#include "planning_context.h"
#include "environmental_model.h"

namespace planning {
ConstructionTakeoverHMIDecider::ConstructionTakeoverHMIDecider(framework::Session *session) {
  session_ = session;
}

bool ConstructionTakeoverHMIDecider::Execute() {
  if (session_ == nullptr) {
    return false;
  }

  if(!HasCipvStaticObs()) {
    return false;
  }
  const auto& ego_state_manager = session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager->ego_v() >= 0.1) {
    return false;
  }
  auto& planning_output = session_->mutable_planning_context()->mutable_planning_output();
  planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_WARRING;
  planning_output.planning_request.request_reason = iflyauto::REQUEST_REASON_ON_CONSTRUCTION;

  return true;
}

bool ConstructionTakeoverHMIDecider::HasCipvStaticObs() {
  const auto& agent_manager = session_->environmental_model().get_agent_manager();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto cipv_agent  = agent_manager->GetAgent(cipv_info.cipv_id());
  if (cipv_agent != nullptr && (cipv_agent->type() == agent::AgentType::TRAFFIC_CONE ||
      cipv_agent->type() == agent::AgentType::TRAFFIC_BARREL ||
      cipv_agent->type() == agent::AgentType::TRAFFIC_TEM_SIGN ||
      cipv_agent->type() == agent::AgentType::WATER_SAFETY_BARRIER ||
      cipv_agent->type() == agent::AgentType::CTASH_BARREL)) {
    return true;
  }

  return false;
}

}  // namespace planning