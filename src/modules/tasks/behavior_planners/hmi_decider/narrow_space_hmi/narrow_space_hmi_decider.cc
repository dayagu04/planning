#include "narrow_space_hmi_decider.h"

#include "environmental_model.h"
#include "planning_context.h"

namespace planning {
namespace {
  constexpr double kEps = 1e-6;
  constexpr double kEgoStopVelThd = 0.1;
  constexpr double kNsaLonDisThred = 50.0;
}
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
  const auto &ego_state = session_->environmental_model().get_ego_state_manager();
  const auto &plannig_init_point = ego_state->planning_init_point();
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
  auto& mutable_nsa_planning_completed =
        session_->mutable_planning_context()->mutable_nsa_planning_completed();
  double nsa_dis = local_view.function_state_machine_info.nra_req.nra_distance;
  if (local_view.function_state_machine_info.current_state == iflyauto::FunctionalState_NRA_GUIDANCE) {
    if (!narrow_space_decider_output.is_exist_narrow_space && !narrow_space_decider_output.is_in_narrow_space &&
        plannig_init_point.v < kEgoStopVelThd) {
      hmi_info->nsa_info.is_complete = true;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NO_NARROW;
      mutable_nsa_planning_completed = true;
    } else if (!narrow_space_decider_output.is_passable_narrow_space && plannig_init_point.v < kEgoStopVelThd) {
      hmi_info->nsa_info.is_complete = true;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NO_SPACE;
      mutable_nsa_planning_completed = true;
    } else if (narrow_space_decider_output.is_passable_narrow_space && narrow_space_decider_output.is_in_narrow_space &&
               plannig_init_point.v < kEgoStopVelThd && nsa_dis > kNsaLonDisThred) {
      hmi_info->nsa_info.is_complete = true;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_DISTANCE_SATISFY;
      mutable_nsa_planning_completed = true;
    } else {
      hmi_info->nsa_info.is_complete = false;
      hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
      mutable_nsa_planning_completed = false;
    }
  } else if (local_view.function_state_machine_info.current_state == iflyauto::FunctionalState_NRA_COMPLETED) {
    hmi_info->nsa_info.is_complete = true;
    hmi_info->nsa_info.nsa_complete_reason = last_complete_reason_;
    mutable_nsa_planning_completed = true;
  } else {
    hmi_info->nsa_info.is_complete = false;
    hmi_info->nsa_info.nsa_complete_reason = iflyauto::NSACompleteReason::NSA_COMPLETE_REASON_NONE;
    mutable_nsa_planning_completed = false;
  }
  last_complete_reason_ = hmi_info->nsa_info.nsa_complete_reason;
  return true;
}

}  // namespace planning