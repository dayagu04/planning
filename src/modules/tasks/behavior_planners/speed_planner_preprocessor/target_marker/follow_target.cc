#include "follow_target.h"

#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "target.h"

namespace planning {

namespace {
  constexpr double kLargeAgentLengthM = 8.0;
}

FollowTarget::FollowTarget(const SpeedPlannerConfig config,
                           framework::Session *session)
    : Target(config, session), config_(config), session_(session) {
  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());
  
}

void FollowTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) {
    return;
  }
  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
    if (upper_bound.agent_id() != speed::kNoAgentId) {
      if (i == 0) {
        cipv_info_.agent_id = upper_bound.agent_id();
        cipv_info_.upper_bound_s = upper_bound.s();
        cipv_info_.vel = upper_bound.velocity();
        auto* agent =
            session_->environmental_model().get_agent_manager()->GetAgent(
                cipv_info_.agent_id);
        if (agent != nullptr) {
          cipv_info_.is_large_vehicle = agent->length() > kLargeAgentLengthM;
          cipv_info_.type = agent->type();
          cipv_info_.is_tfl_virtual_obs = agent->is_tfl_virtual_obs();
        }
      }
      upper_bound_infos_[i].s = upper_bound.s();
      upper_bound_infos_[i].t = t;
      upper_bound_infos_[i].v = upper_bound.velocity();
      upper_bound_infos_[i].target_type = TargetType::kFollow;
      upper_bound_infos_[i].agent_id = upper_bound.agent_id();
      upper_bound_infos_[i].st_boundary_id = upper_bound.boundary_id();
    }
  }
}

}  // planning