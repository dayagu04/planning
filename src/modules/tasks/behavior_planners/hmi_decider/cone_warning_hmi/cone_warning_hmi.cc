#include "cone_warning_hmi.h"
#include "planning_context.h"
#include "environmental_model.h"

namespace planning {
const static int kStartRunningCount = 3;
const static int kStopRunningCount = 3;
ConeWarningHMIDecider::ConeWarningHMIDecider(framework::Session *session) {
  session_ = session;
}

bool ConeWarningHMIDecider::Execute() {
  return false;
  if (session_ == nullptr) {
    return false;
  }
  
  // has_cipv_cone_ = HasCipvCone();
  // has_speed_limit_cone_ = HasSpeedLimitCone();
  // has_alc_cone_ = HasConeALC();

  // switch (current_state_) {
  //   case ConeWarningState::IDLE:
  //     if (IsStartRunning()) {
  //       current_state_ = ConeWarningState::RUNNING;
  //       stop_running_count_ = 0;
  //     }
  //     break;
    
  //   case ConeWarningState::RUNNING:
  //     if (IsStopRunning()) {
  //       current_state_ = ConeWarningState::EXITING;
  //       start_running_count_ = 0;
  //     }
  //     break;
    
  //   case ConeWarningState::EXITING:
  //     current_state_ = ConeWarningState::IDLE;
  //     stop_running_count_ = 0;
  //     stop_running_count_ = 0;
  //     break;
  // }

  SaveHmiOutput();
  return true;
}

bool ConeWarningHMIDecider::HasCipvCone() {
  const auto& agent_manager = session_->environmental_model().get_agent_manager();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto cipv_agent = agent_manager->GetAgent(cipv_info.cipv_id());
  if (cipv_agent != nullptr && cipv_agent->type() == agent::AgentType::TRAFFIC_CONE) {
    return true;
  }

  return false;
}

bool ConeWarningHMIDecider::HasSpeedLimitCone() {
  const auto& agent_manager = session_->environmental_model().get_agent_manager();
  const auto& avoid_speed_limit_info= session_->planning_context().speed_limit_decider_output().avoid_speed_limit_info();
  const auto speed_limit_agent = agent_manager->GetAgent(avoid_speed_limit_info.id);
  if (speed_limit_agent != nullptr && speed_limit_agent->type() == agent::AgentType::TRAFFIC_CONE) {
    return true;
  }

  return false;
}

bool ConeWarningHMIDecider::HasConeALC() {
  const auto state = session_->planning_context().lane_change_decider_output().curr_state;
  bool is_cone_lc =
      session_->planning_context().lane_change_decider_output().lc_request_source == CONE_REQUEST;
  if (state == kLaneChangeExecution && is_cone_lc) {
    return true;
  }
  return false;
}

bool ConeWarningHMIDecider::IsStartRunning() {
  if (has_cipv_cone_ || has_speed_limit_cone_ || has_alc_cone_) {
    start_running_count_ = std::min(kStartRunningCount, start_running_count_ + 1);
  }

  if (start_running_count_ >= kStartRunningCount) {
    return true;
  }
  return false;
}


bool ConeWarningHMIDecider::IsStopRunning() {
  if (!has_cipv_cone_ && !has_speed_limit_cone_ && !has_alc_cone_) {
    stop_running_count_ = std::min(kStartRunningCount, stop_running_count_ + 1);
  }

  if (stop_running_count_ >= kStopRunningCount) {
    return true;
  }
  return false;
}

void ConeWarningHMIDecider::SaveHmiOutput() {
  // auto hmi_info = session_->mutable_planning_context()->mutable_planning_hmi_info();
  // if (current_state_ == ConeWarningState::RUNNING) {
  //   hmi_info->ad_info.cone_warning_info.cone_warning = true;
  // } else {
  //   hmi_info->ad_info.cone_warning_info.cone_warning = false;
  // }
}

}  // namespace planning