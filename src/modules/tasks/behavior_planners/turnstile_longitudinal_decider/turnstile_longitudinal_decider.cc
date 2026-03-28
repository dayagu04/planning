#include "turnstile_longitudinal_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace {
constexpr int32_t kHppTurnstileVirtualAgentId =
    agent::AgentDefaultInfo::kHppTurnstileVirtualAgentId;

void UpdateStableCounter(bool condition, int32_t* counter) {
  if (condition) {
    ++(*counter);
  } else {
    *counter = 0;
  }
}
}  // namespace

TurnstileLongitudinalDecider::TurnstileLongitudinalDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      lon_config_(config_builder->cast<LongitudinalDeciderV3Config>()) {
  name_ = "TurnstileLongitudinalDecider";
  frame_ctx_.stop_virtual_agent_id = kHppTurnstileVirtualAgentId;
}

bool TurnstileLongitudinalDecider::Execute() {
  if (!PreCheck() || !session_->is_hpp_scene()) {
    return false;
  }
  if (!lon_config_.enable_turnstile_longitudinal_decider ||
      !IsCurrentReferencePathValid()) {
    ResetStateWhenDeciderInactive();
    return true;
  }

  InitFrameContextFromReferencePath();
  UpdateTargetTurnstile();
  UpdateFrontVehicle();
  UpdateTurnstilePassability();
  UpdateTurnstileCycleState(frame_ctx_, stage_, cycle_state_, emergency_state_,
                            history_state_);
  UpdateTurnstileStage();

  frame_ctx_.stop_required = ShouldCreateVirtualObstacle();
  frame_ctx_.stop_virtual_agent_id = kHppTurnstileVirtualAgentId;
  if (frame_ctx_.stop_required) {
    AddVirtualObstacle();
  }
  DumpTurnstileDebug();
  return true;
}

bool TurnstileLongitudinalDecider::IsCurrentReferencePathValid() {
  const auto& current_lane =
      session_->environmental_model().get_virtual_lane_manager()->get_current_lane();
  if (current_lane == nullptr) {
    return false;
  }
  reference_path_ = current_lane->get_reference_path();
  return reference_path_ != nullptr;
}

void TurnstileLongitudinalDecider::ResetStateWhenDeciderInactive() {
  frame_ctx_.target_turnstile_frenet_obs = nullptr;
  frame_ctx_.target_turnstile_obs = nullptr;
  frame_ctx_.front_vehicle_frenet_obs = nullptr;
  frame_ctx_.has_target_turnstile = false;
  frame_ctx_.target_turnstile_obs_id = agent::AgentDefaultInfo::kNoAgentId;
  frame_ctx_.side_turnstile_obs_id = agent::AgentDefaultInfo::kNoAgentId;
  frame_ctx_.turnstile_scene_type = TurnstileSceneType::TURNSTILE_SCENE_NONE;
  frame_ctx_.is_head_car = true;
  frame_ctx_.front_car_id = agent::AgentDefaultInfo::kNoAgentId;
  frame_ctx_.turnstile_passable_status = false;
  frame_ctx_.stop_required = false;
  frame_ctx_.stop_virtual_agent_id = kHppTurnstileVirtualAgentId;
  frame_ctx_.turnstile_stop_s = 0.0;
  frame_ctx_.turnstile_s = 0.0;

  cycle_state_.target_turnstile_lost_frame_count = 0;
  cycle_state_.turnstile_passable_status_stable_frame_count = 0;
  cycle_state_.cycle_closing_status_stable_frame_count = 0;
  cycle_state_.reopen_open_status_continuous_frame_count = 0;
  cycle_state_.front_car_passed_in_current_cycle = false;
  cycle_state_.wait_reopen_required = false;
  cycle_state_.has_seen_gate_closed_status_after_front_car_pass = false;
  cycle_state_.release_by_open_timeout = false;

  emergency_state_.closing_status_drop_consecutive_frame_count = 0;
  emergency_state_.emergency_opening_status_stable_frame_count = 0;
  emergency_state_.closing_status_drop_emergency_active = false;

  stage_ = TurnstileStage::IDLE;
}

void TurnstileLongitudinalDecider::InitFrameContextFromReferencePath() {
  frame_ctx_.target_turnstile_frenet_obs = nullptr;
  frame_ctx_.target_turnstile_obs = nullptr;
  frame_ctx_.front_vehicle_frenet_obs = nullptr;
  frame_ctx_.front_car_id = agent::AgentDefaultInfo::kNoAgentId;
  frame_ctx_.is_head_car = true;
  frame_ctx_.has_target_turnstile = false;
  frame_ctx_.turnstile_passable_status = false;

  const auto& turnstile_info = reference_path_->get_turnstile_scene_info();
  frame_ctx_.turnstile_scene_type = turnstile_info.type;
  frame_ctx_.target_turnstile_obs_id = turnstile_info.target_id;
  frame_ctx_.side_turnstile_obs_id = turnstile_info.side_id;

  frame_ctx_.stop_required = false;
  frame_ctx_.stop_virtual_agent_id = kHppTurnstileVirtualAgentId;
  frame_ctx_.turnstile_s = 0.0;
  frame_ctx_.turnstile_stop_s = 0.0;
}

void TurnstileLongitudinalDecider::UpdateTargetTurnstile() {
  const auto mark_target_lost = [this]() {
    ++cycle_state_.target_turnstile_lost_frame_count;
  };

  if (reference_path_ == nullptr) {
    mark_target_lost();
    return;
  }

  const auto& turnstile_info = reference_path_->get_turnstile_scene_info();
  const auto& turnstile_map = reference_path_->get_turnstile_obstacles_map();
  const auto target_turnstile_iter = turnstile_map.find(turnstile_info.target_id);
  const bool has_valid_target_turnstile =
      target_turnstile_iter != turnstile_map.end() &&
      target_turnstile_iter->second != nullptr;
  if (!has_valid_target_turnstile) {
    mark_target_lost();
    return;
  }

  frame_ctx_.target_turnstile_frenet_obs = target_turnstile_iter->second.get();
  frame_ctx_.target_turnstile_obs = frame_ctx_.target_turnstile_frenet_obs->obstacle();
  if (frame_ctx_.target_turnstile_obs == nullptr) {
    mark_target_lost();
    return;
  }

  const double ego_s = reference_path_->get_frenet_ego_state().s();
  const auto& target_boundary =
      frame_ctx_.target_turnstile_frenet_obs->frenet_obstacle_boundary();
  const bool target_is_behind_ego = target_boundary.s_end < ego_s;
  if (target_is_behind_ego) {
    mark_target_lost();
    return;
  }

  cycle_state_.target_turnstile_lost_frame_count = 0;
  frame_ctx_.has_target_turnstile = true;
  frame_ctx_.turnstile_s = target_boundary.s_start;
}

bool TurnstileLongitudinalDecider::IsFrontVehicleNearTurnstile(
    const FrenetObstacle& front_vehicle_frenet_obs) const {
  if (frame_ctx_.target_turnstile_frenet_obs == nullptr) {
    return false;
  }
  const auto& front_vehicle_boundary =
      front_vehicle_frenet_obs.frenet_obstacle_boundary();
  const auto& turnstile_boundary =
      frame_ctx_.target_turnstile_frenet_obs->frenet_obstacle_boundary();
  return front_vehicle_boundary.s_start <=
         turnstile_boundary.s_end + lon_config_.turnstile_near_margin;
}

void TurnstileLongitudinalDecider::UpdateFrontVehicle() {
  frame_ctx_.front_vehicle_frenet_obs = nullptr;
  frame_ctx_.is_head_car = true;
  frame_ctx_.front_car_id = agent::AgentDefaultInfo::kNoAgentId;

  if (reference_path_ == nullptr) {
    return;
  }

  const double ego_s = reference_path_->get_frenet_ego_state().s();
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const int32_t target_id = frame_ctx_.target_turnstile_obs_id;

  int32_t preferred_front_vehicle_id = agent::AgentDefaultInfo::kNoAgentId;
  const auto& dynamic_world = session_->environmental_model().get_dynamic_world();
  if (dynamic_world != nullptr) {
    const int64_t front_node_id = dynamic_world->ego_front_node_id();
    if (front_node_id != -1) {
      const auto* front_node = dynamic_world->GetNode(front_node_id);
      if (front_node != nullptr) {
        preferred_front_vehicle_id = front_node->node_agent_id();
      }
    }
  }
  const bool has_preferred_front_vehicle =
      preferred_front_vehicle_id != agent::AgentDefaultInfo::kNoAgentId;

  const auto& obstacles = reference_path_->get_obstacles();
  double nearest_front_vehicle_s = std::numeric_limits<double>::max();
  for (const auto& obs_ptr : obstacles) {
    if (obs_ptr == nullptr || obs_ptr->obstacle() == nullptr) {
      continue;
    }

    const int32_t obs_id = obs_ptr->id();
    const bool is_target_or_virtual =
        obs_id == target_id || obs_id == kHppTurnstileVirtualAgentId;
    if (is_target_or_virtual) {
      continue;
    }

    if (!obs_ptr->obstacle()->is_car()) {
      continue;
    }

    const auto& boundary = obs_ptr->frenet_obstacle_boundary();
    const bool is_behind_ego = boundary.s_end <= ego_s;
    if (is_behind_ego) {
      continue;
    }

    const bool is_too_far_ahead =
        boundary.s_start - ego_s > lon_config_.turnstile_front_vehicle_max_distance;
    if (is_too_far_ahead) {
      continue;
    }

    const bool has_lateral_overlap =
        !(boundary.l_end < ego_boundary.l_start || boundary.l_start > ego_boundary.l_end);
    if (!has_lateral_overlap) {
      continue;
    }

    if (!IsFrontVehicleNearTurnstile(*obs_ptr)) {
      continue;
    }

    if (has_preferred_front_vehicle && obs_id == preferred_front_vehicle_id) {
      frame_ctx_.front_vehicle_frenet_obs = obs_ptr.get();
      break;
    }

    if (boundary.s_start < nearest_front_vehicle_s) {
      nearest_front_vehicle_s = boundary.s_start;
      frame_ctx_.front_vehicle_frenet_obs = obs_ptr.get();
    }
  }

  if (frame_ctx_.front_vehicle_frenet_obs == nullptr ||
      frame_ctx_.front_vehicle_frenet_obs->obstacle() == nullptr) {
    return;
  }
  frame_ctx_.is_head_car = false;
  frame_ctx_.front_car_id = frame_ctx_.front_vehicle_frenet_obs->id();
}

TurnstileLongitudinalDecider::GateSnapshot
TurnstileLongitudinalDecider::GetGateSnapshot(const Obstacle& turnstile_obs) const {
  GateSnapshot snapshot;
  snapshot.status = turnstile_obs.turnstile_status();
  snapshot.open_ratio = turnstile_obs.turnstile_open_ratio();
  snapshot.is_unknown_or_close =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_UNKNOWN ||
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE;
  snapshot.is_static =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_STATIC;
  snapshot.is_opening =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_OPEN;
  return snapshot;
}
// 比普通 closing 更“宽”，把“静止但开度已很低”也算进来,用途：只给 reopen 周期里“先关后开”观测计数用
bool TurnstileLongitudinalDecider::IsTurnstileInCycleClosingStatus(
    const Obstacle& turnstile_obs) const {
  const GateSnapshot snapshot = GetGateSnapshot(turnstile_obs);
  if (snapshot.is_unknown_or_close) {
    return true;
  }
  return snapshot.is_static &&
         snapshot.open_ratio <=
             lon_config_.turnstile_cycle_closing_status_ratio_threshold;
}

bool TurnstileLongitudinalDecider::IsTurnstileDroppedForEmergency(
    const Obstacle& turnstile_obs) const {
  const GateSnapshot snapshot = GetGateSnapshot(turnstile_obs);
  if (snapshot.is_unknown_or_close) {
    return true;
  }
  return snapshot.open_ratio <=
         lon_config_.turnstile_closing_status_drop_ratio_threshold;
}

double TurnstileLongitudinalDecider::GetEffectiveTurnstileDtSec() const {
  if (lon_config_.turnstile_frame_dt_override_sec > 1e-6) {
    return lon_config_.turnstile_frame_dt_override_sec;
  }
  return std::max(lon_config_.delta_time, 1e-3);
}

bool TurnstileLongitudinalDecider::IsTurnstileInPassableStatus(
    const Obstacle& turnstile_obs) const {
  const GateSnapshot snapshot = GetGateSnapshot(turnstile_obs);
  if (snapshot.is_unknown_or_close) {
    return false;
  }
  if (snapshot.is_static) {
    return snapshot.open_ratio >= lon_config_.turnstile_open_status_threshold;
  }
  if (snapshot.is_opening) {
    return snapshot.open_ratio >= lon_config_.turnstile_passable_status_threshold;
  }
  return false;
}

void TurnstileLongitudinalDecider::UpdateTurnstilePassability() {
  const bool has_target_turnstile_obs = frame_ctx_.target_turnstile_obs != nullptr;
  if (!has_target_turnstile_obs) {
    cycle_state_.turnstile_passable_status_stable_frame_count = 0;
    frame_ctx_.turnstile_passable_status = false;
    return;
  }

  const bool is_turnstile_passable_status =
      IsTurnstileInPassableStatus(*frame_ctx_.target_turnstile_obs);
  UpdateStableCounter(is_turnstile_passable_status,
                      &cycle_state_.turnstile_passable_status_stable_frame_count);
  frame_ctx_.turnstile_passable_status = is_turnstile_passable_status;
}

void TurnstileLongitudinalDecider::UpdateReopenCycleState(
    const FrameContext& frame_ctx, const HistoryState& history_state,
    CycleState& cycle_state) {
  const bool has_target_turnstile_obs = frame_ctx.target_turnstile_obs != nullptr;
  const bool wait_reopen_required = cycle_state.wait_reopen_required;

  const bool can_track_cycle_closing =
      wait_reopen_required && has_target_turnstile_obs &&
      lon_config_.enable_turnstile_cycle_closing_status_reopen_release;
  if (can_track_cycle_closing) {
    const bool is_cycle_closing =
        IsTurnstileInCycleClosingStatus(*frame_ctx.target_turnstile_obs);
    UpdateStableCounter(is_cycle_closing,
                        &cycle_state.cycle_closing_status_stable_frame_count);
    if (cycle_state.cycle_closing_status_stable_frame_count >=
        lon_config_.turnstile_cycle_closing_status_stable_frame_threshold) {
      cycle_state.has_seen_gate_closed_status_after_front_car_pass = true;
    }
  } else {
    cycle_state.cycle_closing_status_stable_frame_count = 0;
  }

  const bool can_track_open_timeout =
      wait_reopen_required && has_target_turnstile_obs &&
      lon_config_.enable_turnstile_open_timeout_release &&
      !cycle_state.has_seen_gate_closed_status_after_front_car_pass;
  if (can_track_open_timeout) {
    UpdateStableCounter(frame_ctx.turnstile_passable_status,
                        &cycle_state.reopen_open_status_continuous_frame_count);
    cycle_state.release_by_open_timeout =
        cycle_state.reopen_open_status_continuous_frame_count *
            GetEffectiveTurnstileDtSec() >=
        lon_config_.turnstile_open_timeout_sec;
  } else {
    cycle_state.reopen_open_status_continuous_frame_count = 0;
    cycle_state.release_by_open_timeout = false;
  }

  const bool has_front_vehicle_in_current_frame =
      frame_ctx.front_vehicle_frenet_obs != nullptr;
  const bool can_detect_front_car_pass =
      !cycle_state.front_car_passed_in_current_cycle &&
      history_state.had_valid_front_vehicle_in_prev_frame &&
      history_state.was_turnstile_passable_status_in_prev_frame &&
      frame_ctx.target_turnstile_frenet_obs != nullptr;
  if (can_detect_front_car_pass) {
    const bool front_car_lost_or_changed =
        !has_front_vehicle_in_current_frame ||
        (frame_ctx.front_vehicle_frenet_obs != nullptr &&
         frame_ctx.front_vehicle_frenet_obs->id() !=
             history_state.previous_front_vehicle_id);

    bool front_car_passed = false;
    if (front_car_lost_or_changed) {
      front_car_passed =
          history_state.previous_front_vehicle_s >=
          frame_ctx.target_turnstile_frenet_obs->frenet_obstacle_boundary().s_start -
              lon_config_.turnstile_passing_window;
    }

    if (front_car_passed) {
      cycle_state.front_car_passed_in_current_cycle = true;
      cycle_state.wait_reopen_required = true;
      cycle_state.has_seen_gate_closed_status_after_front_car_pass = false;
      cycle_state.cycle_closing_status_stable_frame_count = 0;
      cycle_state.reopen_open_status_continuous_frame_count = 0;
      cycle_state.release_by_open_timeout = false;
    }
  }

  if (!cycle_state.wait_reopen_required) {
    cycle_state.cycle_closing_status_stable_frame_count = 0;
    cycle_state.reopen_open_status_continuous_frame_count = 0;
    cycle_state.release_by_open_timeout = false;
  }
}

void TurnstileLongitudinalDecider::UpdateEmergencyState(
    const FrameContext& frame_ctx, TurnstileStage current_stage,
    EmergencyState& emergency_state) {
  const bool has_target_turnstile_obs = frame_ctx.target_turnstile_obs != nullptr;
  const bool is_emergency_stage =
      current_stage == TurnstileStage::PASSING ||
      current_stage == TurnstileStage::EMERGENCY_BLOCK ||
      current_stage == TurnstileStage::PASSABLE_RELEASE;

  const bool can_track_drop_emergency =
      lon_config_.enable_turnstile_closing_status_drop_emergency_stop &&
      has_target_turnstile_obs && is_emergency_stage;
  if (can_track_drop_emergency) {
    const bool is_dropped_for_emergency =
        IsTurnstileDroppedForEmergency(*frame_ctx.target_turnstile_obs);
    UpdateStableCounter(is_dropped_for_emergency,
                        &emergency_state.closing_status_drop_consecutive_frame_count);
    emergency_state.closing_status_drop_emergency_active =
        emergency_state.closing_status_drop_consecutive_frame_count >=
        lon_config_.turnstile_closing_status_drop_consecutive_frame_threshold;
  } else {
    emergency_state.closing_status_drop_consecutive_frame_count = 0;
    emergency_state.closing_status_drop_emergency_active = false;
  }

  const bool can_track_emergency_opening =
      current_stage == TurnstileStage::EMERGENCY_BLOCK && has_target_turnstile_obs;
  if (can_track_emergency_opening) {
    const GateSnapshot snapshot = GetGateSnapshot(*frame_ctx.target_turnstile_obs);
    if (snapshot.is_opening) {
      ++emergency_state.emergency_opening_status_stable_frame_count;
    } else {
      emergency_state.emergency_opening_status_stable_frame_count = 0;
    }
  } else {
    emergency_state.emergency_opening_status_stable_frame_count = 0;
  }
}

void TurnstileLongitudinalDecider::UpdateHistoryFromCurrentFrame(
    const FrameContext& frame_ctx, HistoryState& history_state) {
  if (frame_ctx.front_vehicle_frenet_obs != nullptr) {
    history_state.previous_front_vehicle_id = frame_ctx.front_vehicle_frenet_obs->id();
    history_state.previous_front_vehicle_s =
        frame_ctx.front_vehicle_frenet_obs->frenet_obstacle_boundary().s_end;
    history_state.had_valid_front_vehicle_in_prev_frame = true;
  } else {
    history_state.had_valid_front_vehicle_in_prev_frame = false;
    history_state.previous_front_vehicle_id = agent::AgentDefaultInfo::kNoAgentId;
    history_state.previous_front_vehicle_s = 0.0;
  }
  history_state.was_turnstile_passable_status_in_prev_frame =
      frame_ctx.turnstile_passable_status;
}

void TurnstileLongitudinalDecider::UpdateTurnstileCycleState(
    const FrameContext& frame_ctx, TurnstileStage current_stage,
    CycleState& cycle_state, EmergencyState& emergency_state,
    HistoryState& history_state) {
  UpdateReopenCycleState(frame_ctx, history_state, cycle_state);
  UpdateEmergencyState(frame_ctx, current_stage, emergency_state);
  UpdateHistoryFromCurrentFrame(frame_ctx, history_state);
}

bool TurnstileLongitudinalDecider::HasCompletedReopenCycle(
    const CycleState& cycle_state) const {
  if (!cycle_state.wait_reopen_required) {
    return false;
  }
  const bool completed_by_cycle =
      lon_config_.enable_turnstile_cycle_closing_status_reopen_release &&
      cycle_state.has_seen_gate_closed_status_after_front_car_pass &&
      cycle_state.turnstile_passable_status_stable_frame_count >=
          lon_config_.turnstile_passable_status_stable_frame_threshold;
  const bool completed_by_timeout =
      lon_config_.enable_turnstile_open_timeout_release &&
      cycle_state.release_by_open_timeout;
  return completed_by_cycle || completed_by_timeout;
}

double TurnstileLongitudinalDecider::ComputeTurnstileStopS(
    const FrenetObstacle& turnstile_obs) const {
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const double turnstile_s = turnstile_obs.frenet_obstacle_boundary().s_start;
  return std::max(turnstile_s,
                  ego_boundary.s_end + lon_config_.turnstile_min_forward_stop_buffer);
}

TurnstileLongitudinalDecider::TurnstileEventFlags
TurnstileLongitudinalDecider::BuildTurnstileEventFlags(
    const FrameContext& frame_ctx, const CycleState& cycle_state,
    const EmergencyState& emergency_state,
    const ReferencePath* reference_path) const {
  TurnstileLongitudinalDecider::TurnstileEventFlags flags;

  flags.has_target_turnstile = frame_ctx.has_target_turnstile;
  flags.target_lost_timeout =
      cycle_state.target_turnstile_lost_frame_count >=
      lon_config_.turnstile_target_lost_tolerance_frames;

  flags.is_head_car = frame_ctx.is_head_car;
  flags.turnstile_passable_status = frame_ctx.turnstile_passable_status;
  flags.wait_reopen_required = cycle_state.wait_reopen_required;
  flags.has_seen_gate_closed_status_after_front_car_pass =
      cycle_state.has_seen_gate_closed_status_after_front_car_pass;
  flags.reopen_completed = HasCompletedReopenCycle(cycle_state);
  flags.emergency_active = emergency_state.closing_status_drop_emergency_active;

  const bool passable_status_stable_enough =
      cycle_state.turnstile_passable_status_stable_frame_count >=
      lon_config_.turnstile_passable_status_stable_frame_threshold;
  flags.passable_status_stable =
      flags.turnstile_passable_status && passable_status_stable_enough;

  const double reopen_open_duration_sec =
      cycle_state.reopen_open_status_continuous_frame_count *
      GetEffectiveTurnstileDtSec();
  flags.reopen_timeout_halfway =
      lon_config_.enable_turnstile_open_timeout_release &&
      reopen_open_duration_sec >= 0.5 * lon_config_.turnstile_open_timeout_sec;

  flags.emergency_opening_status_stable =
      emergency_state.emergency_opening_status_stable_frame_count >=
      lon_config_.turnstile_emergency_opening_status_stable_frame_threshold;

  const bool can_evaluate_ego_position =
      reference_path != nullptr && flags.has_target_turnstile;
  if (can_evaluate_ego_position) {
    const auto& ego_boundary = reference_path->get_ego_frenet_boundary();
    const double turnstile_s = frame_ctx.turnstile_s;
    flags.ego_in_gate = ego_boundary.s_end >= turnstile_s;
    flags.ego_passed =
        ego_boundary.s_start > turnstile_s + lon_config_.turnstile_passed_clear_distance;
  }

  const bool has_target_turnstile_obs = frame_ctx.target_turnstile_obs != nullptr;
  if (has_target_turnstile_obs) {
    const GateSnapshot snapshot = GetGateSnapshot(*frame_ctx.target_turnstile_obs);
    flags.gate_opening_status = snapshot.is_opening;
    flags.gate_closed_status =
        snapshot.is_static &&
        snapshot.open_ratio <= lon_config_.turnstile_closed_status_threshold;
    flags.gate_closing_status = snapshot.is_unknown_or_close;
  }

  return flags;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveWaitReopenStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.reopen_completed) {
    return TurnstileStage::PASSABLE_RELEASE;
  }

  const bool is_head_car = flags.is_head_car;
  const bool reopen_timeout_halfway = flags.reopen_timeout_halfway;
  const bool has_seen_gate_closed_after_front_car_pass =
      flags.has_seen_gate_closed_status_after_front_car_pass;
  const bool gate_closing = flags.gate_closing_status;
  const bool gate_closed = flags.gate_closed_status;
  const bool gate_opening = flags.gate_opening_status;

  if (!is_head_car) {
    if (reopen_timeout_halfway) {
      return TurnstileStage::FOLLOW_WAIT_OPEN_TIMEOUT;
    }
    return TurnstileStage::FOLLOW_WAIT_GATE_CLOSE;
  }

  if (!has_seen_gate_closed_after_front_car_pass) {
    if (gate_closing) {
      return TurnstileStage::HEAD_WAIT_CLOSED;
    }
    if (reopen_timeout_halfway) {
      return TurnstileStage::FOLLOW_WAIT_OPEN_TIMEOUT;
    }
    return TurnstileStage::FOLLOW_WAIT_GATE_CLOSE;
  }

  if (gate_closed) {
    return TurnstileStage::HEAD_WAIT_REOPEN;
  }
  if (gate_opening) {
    return TurnstileStage::HEAD_WAIT_FULLY_OPEN;
  }
  if (gate_closing) {
    return TurnstileStage::HEAD_WAIT_CLOSED;
  }
  return TurnstileStage::HEAD_WAIT_REOPEN;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveHeadCarStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.passable_status_stable) {
    return TurnstileStage::PASSABLE_RELEASE;
  }
  if (flags.gate_opening_status) {
    return TurnstileStage::HEAD_WAIT_FULLY_OPEN;
  }
  if (flags.gate_closed_status) {
    return TurnstileStage::HEAD_WAIT_REOPEN;
  }
  if (flags.gate_closing_status) {
    return TurnstileStage::HEAD_WAIT_CLOSED;
  }
  return TurnstileStage::APPROACHING;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveEmergencyExitStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.gate_closed_status) {
    return TurnstileStage::HEAD_WAIT_REOPEN;
  }
  if (flags.emergency_opening_status_stable) {
    return TurnstileStage::HEAD_WAIT_FULLY_OPEN;
  }
  return TurnstileStage::EMERGENCY_BLOCK;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveNextStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags,
    TurnstileStage current_stage) const {
  const bool has_target_turnstile = flags.has_target_turnstile;
  const bool target_lost_timeout = flags.target_lost_timeout;
  if (!has_target_turnstile) {
    return target_lost_timeout ? TurnstileStage::IDLE : current_stage;
  }

  if (flags.ego_passed) {
    return TurnstileStage::PASSED;
  }
  if (flags.emergency_active) {
    return TurnstileStage::EMERGENCY_BLOCK;
  }
  if (flags.ego_in_gate) {
    return TurnstileStage::PASSING;
  }

  const bool was_in_emergency_block = current_stage == TurnstileStage::EMERGENCY_BLOCK;
  if (was_in_emergency_block) {
    return ResolveEmergencyExitStage(flags);
  }

  const bool wait_reopen_required = flags.wait_reopen_required;
  if (wait_reopen_required) {
    return ResolveWaitReopenStage(flags);
  }

  if (!flags.is_head_car) {
    return TurnstileStage::FOLLOW_WAIT;
  }
  return ResolveHeadCarStage(flags);
}

void TurnstileLongitudinalDecider::ResetReopenCycleFlags() {
  cycle_state_.front_car_passed_in_current_cycle = false;
  cycle_state_.wait_reopen_required = false;
  cycle_state_.has_seen_gate_closed_status_after_front_car_pass = false;
  cycle_state_.cycle_closing_status_stable_frame_count = 0;
  cycle_state_.reopen_open_status_continuous_frame_count = 0;
  cycle_state_.release_by_open_timeout = false;
}

void TurnstileLongitudinalDecider::ResetEmergencyFlags() {
  emergency_state_.closing_status_drop_consecutive_frame_count = 0;
  emergency_state_.emergency_opening_status_stable_frame_count = 0;
  emergency_state_.closing_status_drop_emergency_active = false;
}

void TurnstileLongitudinalDecider::OnEnterStage(TurnstileStage prev_stage,
                                                TurnstileStage new_stage,
                                                bool wait_reopen_required) {
  if (prev_stage == new_stage) {
    return;
  }
  if (new_stage == TurnstileStage::IDLE || new_stage == TurnstileStage::PASSED) {
    ResetReopenCycleFlags();
    ResetEmergencyFlags();
    return;
  }
  if (new_stage == TurnstileStage::PASSABLE_RELEASE && wait_reopen_required) {
    ResetReopenCycleFlags();
  }
  if (prev_stage == TurnstileStage::EMERGENCY_BLOCK &&
      new_stage != TurnstileStage::EMERGENCY_BLOCK) {
    ResetEmergencyFlags();
  }
}

void TurnstileLongitudinalDecider::UpdateTurnstileStage() {
  const TurnstileLongitudinalDecider::TurnstileEventFlags flags =
      BuildTurnstileEventFlags(frame_ctx_, cycle_state_, emergency_state_,
                               reference_path_.get());
  const TurnstileStage new_stage = ResolveNextStage(flags, stage_);
  OnEnterStage(stage_, new_stage, cycle_state_.wait_reopen_required);
  stage_ = new_stage;

  if (frame_ctx_.target_turnstile_frenet_obs != nullptr) {
    frame_ctx_.turnstile_stop_s =
        ComputeTurnstileStopS(*frame_ctx_.target_turnstile_frenet_obs);
  }
}

bool TurnstileLongitudinalDecider::ShouldCreateVirtualObstacle() const {
  if (!frame_ctx_.has_target_turnstile) {
    return false;
  }
  if (stage_ == TurnstileStage::EMERGENCY_BLOCK) {
    return true;
  }
  return stage_ != TurnstileStage::IDLE && stage_ != TurnstileStage::PASSABLE_RELEASE &&
         stage_ != TurnstileStage::PASSING && stage_ != TurnstileStage::PASSED;
}

bool TurnstileLongitudinalDecider::AddVirtualObstacle() {
  if (reference_path_ == nullptr || frame_ctx_.target_turnstile_frenet_obs == nullptr) {
    return false;
  }
  ReferencePathPoint ref_point;
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const double turnstile_s =
      frame_ctx_.target_turnstile_frenet_obs->frenet_obstacle_boundary().s_start;
  const double stop_s =
      std::max(turnstile_s + lon_config_.turnstile_stop_buffer,
               ego_boundary.s_end + lon_config_.turnstile_min_forward_stop_buffer);
  if (!reference_path_->get_reference_point_by_lon(stop_s, ref_point)) {
    return false;
  }

  planning::agent::Agent virtual_agent;
  virtual_agent.set_agent_id(kHppTurnstileVirtualAgentId);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(false);
  virtual_agent.set_is_stop_destination_virtual_obs(false);
  virtual_agent.set_is_turnstile_virtual_obs(true);
  virtual_agent.set_x(ref_point.path_point.x());
  virtual_agent.set_y(ref_point.path_point.y());
  virtual_agent.set_theta(ref_point.path_point.theta());
  virtual_agent.set_length(0.5);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});
  planning_math::Box2d box(
      planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());
  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);

  auto* agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);
  return true;
}

void TurnstileLongitudinalDecider::DumpTurnstileDebug() const {
  DEBUG_KEY_VALUE("turnstile_has_target", frame_ctx_.has_target_turnstile)
  DEBUG_KEY_VALUE("turnstile_target_obs_id", frame_ctx_.target_turnstile_obs_id)
  DEBUG_KEY_VALUE("turnstile_side_obs_id", frame_ctx_.side_turnstile_obs_id)
  DEBUG_KEY_VALUE("turnstile_scene_type",
                  static_cast<int32_t>(frame_ctx_.turnstile_scene_type))
  DEBUG_KEY_VALUE("turnstile_stage", static_cast<int32_t>(stage_))
  DEBUG_KEY_VALUE("turnstile_is_head_car", frame_ctx_.is_head_car)
  DEBUG_KEY_VALUE("turnstile_front_car_id", frame_ctx_.front_car_id)
  DEBUG_KEY_VALUE("turnstile_front_car_passed_in_cycle",
                  cycle_state_.front_car_passed_in_current_cycle)
  DEBUG_KEY_VALUE("turnstile_wait_reopen_required", cycle_state_.wait_reopen_required)
  DEBUG_KEY_VALUE(
      "turnstile_seen_closed_status_after_front_pass",
      cycle_state_.has_seen_gate_closed_status_after_front_car_pass)
  DEBUG_KEY_VALUE("turnstile_passable_status", frame_ctx_.turnstile_passable_status)
  DEBUG_KEY_VALUE("turnstile_stop_required", frame_ctx_.stop_required)
  DEBUG_KEY_VALUE("turnstile_virtual_agent_id", frame_ctx_.stop_virtual_agent_id)
  DEBUG_KEY_VALUE("turnstile_s", frame_ctx_.turnstile_s)
  DEBUG_KEY_VALUE("turnstile_stop_s", frame_ctx_.turnstile_stop_s)
  DEBUG_KEY_VALUE("turnstile_target_lost_frame_count",
                  cycle_state_.target_turnstile_lost_frame_count)
  DEBUG_KEY_VALUE("turnstile_passable_status_stable_frame_count",
                  cycle_state_.turnstile_passable_status_stable_frame_count)
  DEBUG_KEY_VALUE("turnstile_cycle_closing_status_stable_frame_count",
                  cycle_state_.cycle_closing_status_stable_frame_count)
  DEBUG_KEY_VALUE("turnstile_reopen_open_status_continuous_frame_count",
                  cycle_state_.reopen_open_status_continuous_frame_count)
  DEBUG_KEY_VALUE("turnstile_release_by_open_timeout",
                  cycle_state_.release_by_open_timeout)
  DEBUG_KEY_VALUE("turnstile_closing_status_drop_consecutive_frame_count",
                  emergency_state_.closing_status_drop_consecutive_frame_count)
  DEBUG_KEY_VALUE("turnstile_closing_status_drop_emergency_active",
                  emergency_state_.closing_status_drop_emergency_active)
  DEBUG_KEY_VALUE("turnstile_emergency_opening_status_stable_frame_count",
                  emergency_state_.emergency_opening_status_stable_frame_count)
}

}  // namespace planning
