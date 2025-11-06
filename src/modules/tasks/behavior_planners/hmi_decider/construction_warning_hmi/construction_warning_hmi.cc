#include "construction_warning_hmi.h"
#include "planning_context.h"

namespace planning {
const static int kStartRunningCount = 3;
const static int kStopRunningCount = 3;
ConstructionWarningHMIDecider::ConstructionWarningHMIDecider(framework::Session *session) {
  session_ = session;
}

bool ConstructionWarningHMIDecider::Execute() {
  return false;
  // if (session_ == nullptr) {
  //   return false;
  // }
  
  // has_construction_ = HasConstruction();

  // switch (current_state_) {
  //   case ConstructionWarningState::IDLE:
  //     if (IsStartRunning()) {
  //       current_state_ = ConstructionWarningState::RUNNING;
  //       stop_running_count_ = 0;
  //     }
  //     break;
    
  //   case ConstructionWarningState::RUNNING:
  //     if (IsStopRunning()) {
  //       current_state_ = ConstructionWarningState::EXITING;
  //       start_running_count_ = 0;
  //     }
  //     break;
    
  //   case ConstructionWarningState::EXITING:
  //     current_state_ = ConstructionWarningState::IDLE;
  //     stop_running_count_ = 0;
  //     stop_running_count_ = 0;
  //     break;
  // }

  SaveHmiOutput();
  return true;
}

bool ConstructionWarningHMIDecider::HasConstruction() {
  const auto& construction_scene_decider_output = session_->planning_context().construction_scene_decider_output();
  if (construction_scene_decider_output.is_exist_construction_area || construction_scene_decider_output.is_pass_construction_area) {
    return true;
  }

  return false;
}


bool ConstructionWarningHMIDecider::IsStartRunning() {
  if (has_construction_) {
    start_running_count_ = std::min(kStartRunningCount, start_running_count_ + 1);
  }

  if (start_running_count_ >= kStartRunningCount) {
    return true;
  }
  return false;
}


bool ConstructionWarningHMIDecider::IsStopRunning() {
  if (!has_construction_) {
    stop_running_count_ = std::min(kStartRunningCount, stop_running_count_ + 1);
  }

  if (stop_running_count_ >= kStopRunningCount) {
    return true;
  }
  return false;
}

void ConstructionWarningHMIDecider::SaveHmiOutput() {
  // auto hmi_info = session_->mutable_planning_context()->mutable_planning_hmi_info();
  // if (current_state_ == ConstructionWarningState::RUNNING) {
  //   if ()
  //   hmi_info->ad_info.construction_info.construction_state  = true;
  // } else {
  //   hmi_info->ad_info.construction_info.construction_state = ConstructionState::CONSTRUCTION_NONE;
  // }
}

}  // namespace planning