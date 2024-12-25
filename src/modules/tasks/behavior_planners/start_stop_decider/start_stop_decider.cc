#include "start_stop_decider.h"

#include "basic_types.pb.h"
#include "behavior_planners/start_stop_decider/start_stop_decider_output.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

StartStopDecider::StartStopDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<StartStopDeciderConfig>()),
      start_stop_status_manager_(config_),
      stop_speed_decision_info_() {
  name_ = "StartStopDecider";
}

bool StartStopDecider::Execute() {
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }
  UpdateInput();
  start_stop_status_manager_.Update();
  StopSpeedDecisionProcess();
  SaveToSession();
  return true;
}

void StartStopDecider::UpdateInput() {
  const bool current_traffic_light_can_pass =
      session_->planning_context().traffic_light_decider_output().can_pass;
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto current_distance_ego_to_stopline =
      virtual_lane_manager->GetEgoDistanceToStopline();
  const auto current_intersection_state =
      virtual_lane_manager->GetIntersectionState();
  const auto& environmental_model = session_->environmental_model();
  const auto& cipv_decider_output =
      session_->planning_context().cipv_decider_output();

  // cipv info
  start_stop_status_manager_.mutable_cipv_id() = cipv_decider_output.cipv_id();
  start_stop_status_manager_.mutable_cipv_relative_s() =
      cipv_decider_output.relative_s();
  start_stop_status_manager_.mutable_cipv_vel_frenet() =
      cipv_decider_output.v_frenet();
  // intersection info
  start_stop_status_manager_.mutable_current_distance_ego_to_stopline() =
      current_distance_ego_to_stopline;
  start_stop_status_manager_.mutable_current_traffic_light_can_pass() =
      current_traffic_light_can_pass;
  start_stop_status_manager_.mutable_current_intersection_state_ego() =
      current_intersection_state;
  // dbw status info
  start_stop_status_manager_.mutable_dbw_status() =
      environmental_model.GetVehicleDbwStatus();
  // ego state info
  start_stop_status_manager_.mutable_planning_init_state_velocity() =
      environmental_model.get_ego_state_manager()
          ->planning_init_point()
          .lon_init_state.v();
  auto& ego_start_stop_info =
      start_stop_status_manager_.mutable_ego_start_stop_info();
  const auto& cur_start_stop_result =
      session_->planning_context().start_stop_result();
  ego_start_stop_info.CopyFrom(cur_start_stop_result);
}

void StartStopDecider::StopSpeedDecisionProcess() {
  if (start_stop_status_manager_.ego_start_stop_info().state() ==
      common::StartStopInfo::STOP) {
    stop_speed_decision_info_.mutable_is_valid() = true;
    stop_speed_decision_info_.mutable_s() = 0.0;
    stop_speed_decision_info_.mutable_v() = 0.0;
    stop_speed_decision_info_.mutable_a() = 0.0;
  } else {
    stop_speed_decision_info_.mutable_is_valid() = false;
  }
}

void StartStopDecider::SaveToSession() {
  auto& start_stop_decider_output =
      session_->mutable_planning_context()->mutable_start_stop_decider_output();
  start_stop_decider_output.mutable_ego_start_stop_info() =
      start_stop_status_manager_.ego_start_stop_info();
   start_stop_decider_output.mutable_stop_speed_decision_info() =
      stop_speed_decision_info_; 

  auto& start_stop_state_result =
      session_->mutable_planning_context()->mutable_start_stop_result();
  start_stop_state_result.CopyFrom(
      start_stop_status_manager_.ego_start_stop_info());
}

}  // namespace planning