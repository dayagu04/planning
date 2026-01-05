#include "obstacle_brake_hmi_decider.h"
#include "local_view.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "agent/agent.h"

namespace planning {

ObstacleBrakeHMIDecider::ObstacleBrakeHMIDecider(framework::Session* session,
                                                 const HmiDeciderConfig& config)
                                                 : session_(session), config_(config) {}

bool ObstacleBrakeHMIDecider::Execute() {
  if (session_->is_rads_scene()) {
    GenerateHMIInfoForRADS();
  } else {
    // ad info
  }
  return true;
}

bool ObstacleBrakeHMIDecider::GenerateHMIInfoForRADS() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();

  const auto cipv_info = session_->planning_context().cipv_decider_output();
  const auto &local_view = session_->environmental_model().get_local_view();
  auto fsm_state = local_view.function_state_machine_info.current_state;

  if(fsm_state == iflyauto::FunctionalState_RADS_TRACING && cipv_info.cipv_id() != -1 &&
     cipv_info.cipv_id() != agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId &&
     cipv_info.relative_s() < config_.obstacle_brake_hmi_reminder_dis) {
    hmi_info->rads_info.rads_pause_reason = iflyauto::RADSPauseReason::RADS_PAUSE_REASON_BRAKE_BY_OBSTACLE;
  } else {
    hmi_info->rads_info.rads_pause_reason = iflyauto::RADSPauseReason::RADS_PAUSE_REASON_NONE;
  }
  JSON_DEBUG_VALUE("obstacle_brake_hmi_sts", int(hmi_info->rads_info.rads_pause_reason));
  return true;
}

}  // namespace planning