#include "narrow_space_hmi_decider.h"

#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

NarrowSpaceHMIDecider::NarrowSpaceHMIDecider(framework::Session* session) {
  session_ = session;
  last_complete_reason_ = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
}

bool NarrowSpaceHMIDecider::Execute() {
  // init
  InitInfo();
  // assemble
  GenerateHMIInfo();
  return true;
}

void NarrowSpaceHMIDecider::InitInfo() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  hmi_info->nsa_info.is_avaliable = false;
  hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_NOT_ON_NARROW;
  hmi_info->nsa_info.is_complete = false;
  hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
}

bool NarrowSpaceHMIDecider::GenerateHMIInfo() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  const auto& narrow_space_decider_output =
      session_->planning_context().narrow_space_decider_output();
  bool is_passable_condition = narrow_space_decider_output.is_in_narrow_space;
  if (!narrow_space_decider_output.is_in_narrow_space) {
    hmi_info->nsa_info.is_avaliable = false;
    hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_NOT_ON_NARROW;
  } else if (narrow_space_decider_output.is_too_narrow) {
    hmi_info->nsa_info.is_avaliable = false;
    hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_NARROW;
  } else if (narrow_space_decider_output.is_too_wide) {
    hmi_info->nsa_info.is_avaliable = false;
    hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_WIDE;
  } else if (narrow_space_decider_output.is_relative_angle_too_large) {
    hmi_info->nsa_info.is_avaliable = false;
    hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_HEADING;
  } else {
    hmi_info->nsa_info.is_avaliable = true;
    hmi_info->nsa_info.nsa_disable_reason = iflyauto::NSADisableReason::NSA_DISABLE_REASON_NONE;
  }
  const auto& local_view = session_->environmental_model().get_local_view();
  if (local_view.function_state_machine_info.current_state == iflyauto::FunctionalState_NRA_GUIDANCE) {
    if (!narrow_space_decider_output.is_exist_narrow_space) {
      hmi_info->nsa_info.is_complete = true;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NO_NARROW;
    } else if (!narrow_space_decider_output.is_passable_narrow_space) {
      hmi_info->nsa_info.is_complete = true;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NO_SPACE;
    } else {
      hmi_info->nsa_info.is_complete = false;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
    }
  } else if (local_view.function_state_machine_info.current_state == iflyauto::FunctionalState_NRA_COMPLETED) {
    hmi_info->nsa_info.is_complete = true;
    hmi_info->nsa_info.nsa_complete_reason = last_complete_reason_;
  } else {
    hmi_info->nsa_info.is_complete = false;
    hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
  }
  last_complete_reason_ = hmi_info->nsa_info.nsa_complete_reason;
  return true;
}

}  // namespace planning