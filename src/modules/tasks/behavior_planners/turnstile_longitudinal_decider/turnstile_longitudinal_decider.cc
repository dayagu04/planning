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

  // 每帧执行顺序：输入刷新 -> 跨帧状态更新 -> 状态机转移 -> 输出停车决策。
  InitFrameContextFromReferencePath();
  UpdateTargetTurnstile();
  UpdateGateSnapshot();
  UpdateFrontVehicle();
  UpdateTurnstilePassability();
  UpdateTurnstileCrossFrameStates();
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
  frame_ctx_.turnstile_passable_status_stable = false;
  frame_ctx_.stop_required = false;
  frame_ctx_.stop_virtual_agent_id = kHppTurnstileVirtualAgentId;
  frame_ctx_.turnstile_stop_s = 0.0;
  frame_ctx_.turnstile_s = 0.0;

  cycle_state_.target_turnstile_lost_frame_count = 0;
  cycle_state_.target_lost_timeout = false;
  cycle_state_.turnstile_passable_status_stable_frame_count = 0;
  cycle_state_.reopen_open_status_continuous_frame_count = 0;
  cycle_state_.front_car_passed_in_current_cycle = false;
  cycle_state_.wait_reopen_after_front_car_passed = false;
  cycle_state_.release_by_open_timeout = false;

  emergency_state_.closing_status_drop_consecutive_frame_count = 0;
  emergency_state_.emergency_stop_stable_frame_count = 0;
  emergency_state_.closing_status_drop_emergency_active = false;
  emergency_state_.emergency_stop_stable = false;

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
  frame_ctx_.turnstile_passable_status_stable = false;

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
  // 统一的“目标丢失”处理：累计丢失帧并在达到阈值后触发 timeout。
  const auto mark_target_lost = [this]() {
    ++cycle_state_.target_turnstile_lost_frame_count;
    cycle_state_.target_lost_timeout =
        cycle_state_.target_turnstile_lost_frame_count >=
        lon_config_.turnstile_target_lost_tolerance_frames;
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
  cycle_state_.target_lost_timeout = false;
  frame_ctx_.has_target_turnstile = true;
  frame_ctx_.turnstile_s = target_boundary.s_start;
}

void TurnstileLongitudinalDecider::UpdateGateSnapshot() {
  if (!frame_ctx_.has_target_turnstile || frame_ctx_.target_turnstile_obs == nullptr) {
    return;
  }
  frame_ctx_.target_turnstile_gate_snapshot =
      GetGateSnapshot(*frame_ctx_.target_turnstile_obs);
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
  // 前车选择策略：优先动态世界 front_node，其次选同车道最近前车（且需靠近道闸）。
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
  snapshot.is_static =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_STATIC;

  const bool motion_dir_open =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_OPEN;
  const bool motion_dir_close =
      snapshot.status == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE;

  snapshot.is_closed =
      snapshot.is_static &&
      snapshot.open_ratio < lon_config_.turnstile_closed_status_threshold;
  snapshot.is_opened =
      snapshot.is_static &&
      snapshot.open_ratio > lon_config_.turnstile_open_status_threshold;

  snapshot.is_opening =
      motion_dir_open ||
      (snapshot.is_static &&
       snapshot.open_ratio > lon_config_.turnstile_closed_status_threshold);
  snapshot.is_closing =
      motion_dir_close ||
      (snapshot.is_static &&
       snapshot.open_ratio < lon_config_.turnstile_open_status_threshold);

  snapshot.is_passable =
      snapshot.open_ratio >= lon_config_.turnstile_passable_status_threshold &&
      (snapshot.is_static || motion_dir_open);
  return snapshot;
}


void TurnstileLongitudinalDecider::UpdateTurnstilePassability() {
  // 维护可通行稳定计数：用于状态机放行判定。
  if (!frame_ctx_.has_target_turnstile) {
    cycle_state_.turnstile_passable_status_stable_frame_count = 0;
    frame_ctx_.turnstile_passable_status = false;
    frame_ctx_.turnstile_passable_status_stable = false;
    return;
  }

  const bool is_turnstile_passable_status =
      frame_ctx_.target_turnstile_gate_snapshot.is_passable;
  UpdateStableCounter(is_turnstile_passable_status,
                      &cycle_state_.turnstile_passable_status_stable_frame_count);

  frame_ctx_.turnstile_passable_status = is_turnstile_passable_status;
  frame_ctx_.turnstile_passable_status_stable =
      is_turnstile_passable_status &&
      cycle_state_.turnstile_passable_status_stable_frame_count >=
          lon_config_.turnstile_passable_status_stable_frame_threshold;
}

void TurnstileLongitudinalDecider::UpdateTurnstileCrossFrameStates() {
  // 跨帧状态维护：reopen 周期、open-timeout 放行、紧急阻挡触发/解除、前车历史缓存。
  const bool has_target_turnstile_snapshot = frame_ctx_.has_target_turnstile;

  // open-timeout 只在 reopen 等待周期内生效；达到 opened 连续帧阈值后可超时放行。
  const bool can_track_open_timeout =
      cycle_state_.wait_reopen_after_front_car_passed && has_target_turnstile_snapshot &&
      lon_config_.enable_turnstile_open_timeout_release;
  if (can_track_open_timeout) {
    const bool is_gate_opened = frame_ctx_.target_turnstile_gate_snapshot.is_opened;
    UpdateStableCounter(is_gate_opened,
                        &cycle_state_.reopen_open_status_continuous_frame_count);
    cycle_state_.release_by_open_timeout =
        cycle_state_.reopen_open_status_continuous_frame_count >=
        lon_config_.turnstile_reopen_timeout_opened_frame_threshold;
  } else {
    cycle_state_.reopen_open_status_continuous_frame_count = 0;
    cycle_state_.release_by_open_timeout = false;
  }

  const bool has_front_vehicle_in_current_frame =
      frame_ctx_.front_vehicle_frenet_obs != nullptr;
  // “前车通过”判定：上一帧有有效前车且本帧丢失/换车，并且上一帧处于可通行窗口。
  const bool can_detect_front_car_pass =
      !cycle_state_.front_car_passed_in_current_cycle &&
      history_state_.had_valid_front_vehicle_in_prev_frame &&
      history_state_.was_turnstile_passable_status_in_prev_frame &&
      frame_ctx_.target_turnstile_frenet_obs != nullptr;
  if (can_detect_front_car_pass) {
    const bool front_car_lost_or_changed =
        !has_front_vehicle_in_current_frame ||
        (frame_ctx_.front_vehicle_frenet_obs != nullptr &&
         frame_ctx_.front_vehicle_frenet_obs->id() !=
             history_state_.previous_front_vehicle_id);

    bool front_car_passed = false;
    if (front_car_lost_or_changed) {
      front_car_passed =
          history_state_.previous_front_vehicle_s >=
          frame_ctx_.target_turnstile_frenet_obs->frenet_obstacle_boundary().s_start -
              lon_config_.turnstile_passing_window;
    }

    if (front_car_passed) {
      cycle_state_.front_car_passed_in_current_cycle = true;
      // 前车通过后进入 reopen 等待周期，相关计数从头开始。
      cycle_state_.wait_reopen_after_front_car_passed = true;
      cycle_state_.reopen_open_status_continuous_frame_count = 0;
      cycle_state_.release_by_open_timeout = false;
    }
  }

  if (!cycle_state_.wait_reopen_after_front_car_passed) {
    // 未处于 reopen 等待时，不保留 reopen/timeout 相关计数。
    cycle_state_.reopen_open_status_continuous_frame_count = 0;
    cycle_state_.release_by_open_timeout = false;
  }

  const bool is_drop_emergency_monitor_stage =
      stage_ == TurnstileStage::PASSING || stage_ == TurnstileStage::EMERGENCY_BLOCK ||
      stage_ == TurnstileStage::PASSABLE_RELEASE;
  const bool can_track_drop_emergency =
      lon_config_.enable_turnstile_closing_status_drop_emergency_stop &&
      has_target_turnstile_snapshot && is_drop_emergency_monitor_stage;
  if (can_track_drop_emergency) {
    const bool is_dropped_for_emergency =
        !frame_ctx_.target_turnstile_gate_snapshot.is_opened;
    UpdateStableCounter(is_dropped_for_emergency,
                        &emergency_state_.closing_status_drop_consecutive_frame_count);
    emergency_state_.closing_status_drop_emergency_active =
        emergency_state_.closing_status_drop_consecutive_frame_count >=
        lon_config_.turnstile_closing_status_drop_consecutive_frame_threshold;
  } else {
    emergency_state_.closing_status_drop_consecutive_frame_count = 0;
    emergency_state_.closing_status_drop_emergency_active = false;
  }

  const bool can_track_emergency_stop_stable = stage_ == TurnstileStage::EMERGENCY_BLOCK;
  if (can_track_emergency_stop_stable && reference_path_ != nullptr) {
    const double ego_velocity_abs =
        std::fabs(session_->environmental_model().get_ego_state_manager()->ego_v());
    const bool ego_stop_stable =
        ego_velocity_abs <= lon_config_.turnstile_emergency_stop_velocity_threshold;
    UpdateStableCounter(ego_stop_stable,
                        &emergency_state_.emergency_stop_stable_frame_count);
    emergency_state_.emergency_stop_stable =
        emergency_state_.emergency_stop_stable_frame_count >=
        lon_config_.turnstile_emergency_stop_stable_frame_threshold;
  } else {
    emergency_state_.emergency_stop_stable_frame_count = 0;
    emergency_state_.emergency_stop_stable = false;
  }

  if (frame_ctx_.front_vehicle_frenet_obs != nullptr) {
    history_state_.previous_front_vehicle_id = frame_ctx_.front_vehicle_frenet_obs->id();
    history_state_.previous_front_vehicle_s =
        frame_ctx_.front_vehicle_frenet_obs->frenet_obstacle_boundary().s_end;
    history_state_.had_valid_front_vehicle_in_prev_frame = true;
  } else {
    history_state_.had_valid_front_vehicle_in_prev_frame = false;
    history_state_.previous_front_vehicle_id = agent::AgentDefaultInfo::kNoAgentId;
    history_state_.previous_front_vehicle_s = 0.0;
  }
  history_state_.was_turnstile_passable_status_in_prev_frame =
      frame_ctx_.turnstile_passable_status;
}

bool TurnstileLongitudinalDecider::IsOpenTimeoutReleaseReady(
    const CycleState& cycle_state) const {
  if (!cycle_state.wait_reopen_after_front_car_passed) {
    return false;
  }
  return lon_config_.enable_turnstile_open_timeout_release &&
         cycle_state.release_by_open_timeout;
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
  // 将 frame/cycle/emergency 的离散信号汇总为状态机判定事件。
  TurnstileLongitudinalDecider::TurnstileEventFlags flags;

  flags.has_target_turnstile = frame_ctx.has_target_turnstile;
  flags.target_lost_timeout = cycle_state.target_lost_timeout;
  flags.is_head_car = frame_ctx.is_head_car;
  flags.wait_reopen_required =
      cycle_state.wait_reopen_after_front_car_passed;
  flags.open_timeout_release_ready = IsOpenTimeoutReleaseReady(cycle_state);
  flags.emergency_active = emergency_state.closing_status_drop_emergency_active;
  flags.head_release_opened_stable = frame_ctx.turnstile_passable_status_stable;
  flags.emergency_stop_stable = emergency_state.emergency_stop_stable;

  const bool has_reference_path_and_target_turnstile =
      reference_path != nullptr && flags.has_target_turnstile;
  if (has_reference_path_and_target_turnstile) {
    const auto& ego_boundary = reference_path->get_ego_frenet_boundary();
    const double turnstile_s = frame_ctx.turnstile_s;
    flags.ego_in_gate =
        ego_boundary.s_end >= turnstile_s - lon_config_.turnstile_ego_in_gate_margin;
    flags.ego_passed =
        ego_boundary.s_start > turnstile_s + lon_config_.turnstile_passed_clear_distance;
  }

  if (frame_ctx.has_target_turnstile) {
    const GateSnapshot& snapshot = frame_ctx.target_turnstile_gate_snapshot;
    flags.gate_opening_status = snapshot.is_opening;
    flags.gate_closed_status = snapshot.is_closed;
    flags.gate_closing_status = snapshot.is_closing;
  }

  return flags;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveWaitReopenStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  return flags.is_head_car ? TurnstileStage::FOLLOW_WAIT_GATE_CLOSE
                           : TurnstileStage::FOLLOW_WAIT;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveFollowWaitGateCloseStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.open_timeout_release_ready) {
    return TurnstileStage::PASSABLE_RELEASE;
  }
  if (flags.gate_closed_status) {
    return TurnstileStage::HEAD_WAIT_REOPEN;
  }
  if (flags.gate_opening_status) {
    return TurnstileStage::HEAD_WAIT_FULLY_OPEN;
  }
  if (flags.gate_closing_status) {
    return TurnstileStage::HEAD_WAIT_CLOSED;
  }
  return TurnstileStage::FOLLOW_WAIT_GATE_CLOSE;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveHeadCarStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.head_release_opened_stable) {
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
  if (!flags.emergency_stop_stable) {
    return TurnstileStage::EMERGENCY_BLOCK;
  }
  if (flags.gate_closed_status) {
    return TurnstileStage::HEAD_WAIT_REOPEN;
  }
  if (flags.gate_opening_status) {
    return TurnstileStage::HEAD_WAIT_FULLY_OPEN;
  }
  return TurnstileStage::APPROACHING;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveNextStageByGraph(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags,
    TurnstileStage current_stage) const {
  // 状态机总入口：先处理全局高优先级条件，再按当前状态走图转移。
  if (!flags.has_target_turnstile) {
    // 目标丢失：仅当超过容忍帧数才回 IDLE，否则保持当前状态。
    return flags.target_lost_timeout ? TurnstileStage::IDLE : current_stage;
  }

  if (flags.ego_passed) {
    return TurnstileStage::PASSED;
  }

  if (current_stage == TurnstileStage::EMERGENCY_BLOCK) {
    return ResolveEmergencyExitStage(flags);
  }

  const bool can_enter_emergency =
      current_stage == TurnstileStage::PASSABLE_RELEASE ||
      current_stage == TurnstileStage::PASSING;
  if (can_enter_emergency && flags.emergency_active) {
    return TurnstileStage::EMERGENCY_BLOCK;
  }

  if (flags.ego_in_gate) {
    // 一旦车头进入闸机区域，优先切换到 PASSING。
    return TurnstileStage::PASSING;
  }

  switch (current_stage) {
    case TurnstileStage::IDLE:
      return TurnstileStage::APPROACHING;

    case TurnstileStage::APPROACHING:
      if (flags.wait_reopen_required) {
        return ResolveWaitReopenStage(flags);
      }
      if (!flags.is_head_car) {
        return TurnstileStage::FOLLOW_WAIT;
      }
      return ResolveHeadCarStage(flags);

    case TurnstileStage::FOLLOW_WAIT:
      if (flags.wait_reopen_required) {
        return ResolveWaitReopenStage(flags);
      }
      if (flags.is_head_car) {
        return TurnstileStage::APPROACHING;
      }
      return TurnstileStage::FOLLOW_WAIT;

    case TurnstileStage::FOLLOW_WAIT_GATE_CLOSE:
      return ResolveFollowWaitGateCloseStage(flags);

    case TurnstileStage::HEAD_WAIT_CLOSED:
    case TurnstileStage::HEAD_WAIT_REOPEN:
    case TurnstileStage::HEAD_WAIT_FULLY_OPEN:
      // 头车等待链路统一按闸机状态推进。
      return ResolveHeadCarStage(flags);

    case TurnstileStage::PASSABLE_RELEASE:
      return TurnstileStage::PASSABLE_RELEASE;

    case TurnstileStage::PASSING:
      return TurnstileStage::PASSING;

    case TurnstileStage::EMERGENCY_BLOCK:
      return ResolveEmergencyExitStage(flags);

    case TurnstileStage::PASSED:
      return TurnstileStage::PASSED;
  }

  return current_stage;
}

void TurnstileLongitudinalDecider::ResetReopenCycleFlags() {
  cycle_state_.front_car_passed_in_current_cycle = false;
  cycle_state_.wait_reopen_after_front_car_passed = false;
  cycle_state_.reopen_open_status_continuous_frame_count = 0;
  cycle_state_.release_by_open_timeout = false;
}

void TurnstileLongitudinalDecider::ResetEmergencyFlags() {
  emergency_state_.closing_status_drop_consecutive_frame_count = 0;
  emergency_state_.emergency_stop_stable_frame_count = 0;
  emergency_state_.closing_status_drop_emergency_active = false;
  emergency_state_.emergency_stop_stable = false;
}

void TurnstileLongitudinalDecider::ResetStatesOnStageTransition(
    TurnstileStage prev_stage, TurnstileStage new_stage,
    bool wait_reopen_after_front_car_passed) {
  // 仅在状态变化时做重置：到终态全量清理，到 PASSABLE_RELEASE 按 reopen 标记清理周期。
  if (prev_stage == new_stage) {
    return;
  }
  if (new_stage == TurnstileStage::IDLE || new_stage == TurnstileStage::PASSED) {
    // 到终态后清空 reopen 和 emergency 的跨帧状态。
    ResetReopenCycleFlags();
    ResetEmergencyFlags();
    return;
  }
  if (new_stage == TurnstileStage::PASSABLE_RELEASE &&
      wait_reopen_after_front_car_passed) {
    ResetReopenCycleFlags();
  }
  if (prev_stage == TurnstileStage::EMERGENCY_BLOCK &&
      new_stage != TurnstileStage::EMERGENCY_BLOCK) {
    ResetEmergencyFlags();
  }
}

void TurnstileLongitudinalDecider::UpdateTurnstileStage() {
  // 基于当前事件计算下一状态，并在切换后执行对应重置副作用。
  const TurnstileLongitudinalDecider::TurnstileEventFlags flags =
      BuildTurnstileEventFlags(frame_ctx_, cycle_state_, emergency_state_,
                               reference_path_.get());
  //根据状态图解析下一状态
  const TurnstileStage new_stage = ResolveNextStageByGraph(flags, stage_);
  ResetStatesOnStageTransition(stage_, new_stage,
                               cycle_state_.wait_reopen_after_front_car_passed);
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
  auto& planning_debug_info = DebugInfoManager::GetInstance().GetDebugInfoPb();
  if (planning_debug_info == nullptr) {
    return;
  }

  auto* turnstile_debug = planning_debug_info->mutable_turnstile_longitudinal_debug();
  turnstile_debug->set_stage(static_cast<int32_t>(stage_));

  turnstile_debug->set_has_target_turnstile(frame_ctx_.has_target_turnstile);
  turnstile_debug->set_target_turnstile_obs_id(frame_ctx_.target_turnstile_obs_id);
  turnstile_debug->set_side_turnstile_obs_id(frame_ctx_.side_turnstile_obs_id);
  turnstile_debug->set_scene_type(
      static_cast<int32_t>(frame_ctx_.turnstile_scene_type));
  turnstile_debug->set_is_head_car(frame_ctx_.is_head_car);
  turnstile_debug->set_front_car_id(frame_ctx_.front_car_id);
  turnstile_debug->set_turnstile_passable_status(
      frame_ctx_.turnstile_passable_status);

  turnstile_debug->set_stop_required(frame_ctx_.stop_required);
  turnstile_debug->set_stop_virtual_agent_id(frame_ctx_.stop_virtual_agent_id);
  turnstile_debug->set_turnstile_s(frame_ctx_.turnstile_s);
  turnstile_debug->set_turnstile_stop_s(frame_ctx_.turnstile_stop_s);

  turnstile_debug->set_front_car_passed_in_current_cycle(
      cycle_state_.front_car_passed_in_current_cycle);
  turnstile_debug->set_wait_reopen_required(cycle_state_.wait_reopen_after_front_car_passed);
  turnstile_debug->set_release_by_open_timeout(cycle_state_.release_by_open_timeout);
  turnstile_debug->set_closing_status_drop_emergency_active(
      emergency_state_.closing_status_drop_emergency_active);

  turnstile_debug->set_previous_front_vehicle_id(
      history_state_.previous_front_vehicle_id);
  turnstile_debug->set_previous_front_vehicle_s(history_state_.previous_front_vehicle_s);
  turnstile_debug->set_had_valid_front_vehicle_in_prev_frame(
      history_state_.had_valid_front_vehicle_in_prev_frame);
  turnstile_debug->set_was_turnstile_passable_status_in_prev_frame(
      history_state_.was_turnstile_passable_status_in_prev_frame);

  const TurnstileLongitudinalDecider::TurnstileEventFlags flags =
      BuildTurnstileEventFlags(frame_ctx_, cycle_state_, emergency_state_,
                               reference_path_.get());
  auto* event_flags = turnstile_debug->mutable_event_flags();
  event_flags->set_has_target_turnstile(flags.has_target_turnstile);
  event_flags->set_target_lost_timeout(flags.target_lost_timeout);
  event_flags->set_ego_passed(flags.ego_passed);
  event_flags->set_ego_in_gate(flags.ego_in_gate);
  event_flags->set_emergency_active(flags.emergency_active);
  event_flags->set_emergency_stop_stable(flags.emergency_stop_stable);
  event_flags->set_wait_reopen_required(flags.wait_reopen_required);
  event_flags->set_open_timeout_release_ready(flags.open_timeout_release_ready);
  event_flags->set_is_head_car(flags.is_head_car);
  event_flags->set_gate_opening_status(flags.gate_opening_status);
  event_flags->set_gate_closed_status(flags.gate_closed_status);
  event_flags->set_gate_closing_status(flags.gate_closing_status);

  auto* counters = turnstile_debug->mutable_counters();
  counters->set_target_turnstile_lost_frame_count(
      cycle_state_.target_turnstile_lost_frame_count);
  counters->set_turnstile_passable_status_stable_frame_count(
      cycle_state_.turnstile_passable_status_stable_frame_count);
  counters->set_reopen_open_status_continuous_frame_count(
      cycle_state_.reopen_open_status_continuous_frame_count);
  counters->set_closing_status_drop_consecutive_frame_count(
      emergency_state_.closing_status_drop_consecutive_frame_count);
  counters->set_emergency_stop_stable_frame_count(
      emergency_state_.emergency_stop_stable_frame_count);

  const GateSnapshot& snapshot = frame_ctx_.target_turnstile_gate_snapshot;
  turnstile_debug->set_gate_status_raw(static_cast<int32_t>(snapshot.status));
  turnstile_debug->set_gate_open_ratio_raw(snapshot.open_ratio);
  turnstile_debug->set_gate_snapshot_is_static(snapshot.is_static);
  turnstile_debug->set_gate_snapshot_is_opening(snapshot.is_opening);
  turnstile_debug->set_gate_snapshot_is_closing(snapshot.is_closing);
  turnstile_debug->set_gate_snapshot_is_closed(snapshot.is_closed);
  turnstile_debug->set_gate_snapshot_is_opened(snapshot.is_opened);
  turnstile_debug->set_gate_snapshot_is_passable(snapshot.is_passable);
}

}  // namespace planning
