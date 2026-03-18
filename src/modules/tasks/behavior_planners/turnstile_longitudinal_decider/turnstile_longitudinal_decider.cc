#include "turnstile_longitudinal_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace {
constexpr int32_t kHppTurnstileVirtualAgentId =
    agent::AgentDefaultInfo::kHppTurnstileVirtualAgentId;
}  // namespace

TurnstileLongitudinalDecider::TurnstileLongitudinalDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      lon_config_(config_builder->cast<LongitudinalDeciderV3Config>()) {
  name_ = "TurnstileLongitudinalDecider";
  turnstile_decider_output_.Clear();
  turnstile_decider_output_.set_stop_virtual_agent_id(kHppTurnstileVirtualAgentId);
}

bool TurnstileLongitudinalDecider::Execute() {
  if (!PreCheck() || !session_->is_hpp_scene()) {
    return false;
  }
  if (!lon_config_.enable_turnstile_longitudinal_decider ||
      !UpdateCurrentReferencePath()) {
    turnstile_decider_output_.Clear();
    turnstile_decider_output_.set_stop_virtual_agent_id(kHppTurnstileVirtualAgentId);
    SaveToSession();
    return true;
  }

  ResetCurrentFrameState();
  UpdateTargetTurnstile();
  UpdateFrontVehicle();
  UpdateTurnstilePassability();
  UpdateTurnstileCycleState();
  UpdateTurnstileStage();
  turnstile_decider_output_.set_stop_required(ShouldCreateVirtualObstacle());
  turnstile_decider_output_.set_stop_virtual_agent_id(kHppTurnstileVirtualAgentId);
  if (turnstile_decider_output_.stop_required()) {
    AddVirtualObstacle();
  }
  JSON_DEBUG_VALUE("turnstile_has_target", turnstile_decider_output_.has_target_turnstile())
  JSON_DEBUG_VALUE("turnstile_target_obs_id", turnstile_decider_output_.target_turnstile_obs_id())
  JSON_DEBUG_VALUE("turnstile_side_obs_id", turnstile_decider_output_.side_turnstile_obs_id())
  JSON_DEBUG_VALUE("turnstile_scene_type", turnstile_decider_output_.turnstile_scene_type())
  JSON_DEBUG_VALUE("turnstile_stage", turnstile_decider_output_.turnstile_stage())
  JSON_DEBUG_VALUE("turnstile_is_head_car", turnstile_decider_output_.is_head_car())
  JSON_DEBUG_VALUE("turnstile_front_car_id", turnstile_decider_output_.front_car_id())
  JSON_DEBUG_VALUE("turnstile_front_car_passed_in_cycle",
                   turnstile_decider_output_.front_car_passed_in_current_cycle())
  JSON_DEBUG_VALUE("turnstile_wait_reopen_required",
                   turnstile_decider_output_.wait_reopen_required())
  JSON_DEBUG_VALUE("turnstile_seen_closed_status_after_front_pass",
                   turnstile_decider_output_.has_seen_gate_closed_status_after_front_car_pass())
  JSON_DEBUG_VALUE("turnstile_passable_status", turnstile_decider_output_.turnstile_passable_status())
  JSON_DEBUG_VALUE("turnstile_stop_required", turnstile_decider_output_.stop_required())
  JSON_DEBUG_VALUE("turnstile_virtual_agent_id", turnstile_decider_output_.stop_virtual_agent_id())
  JSON_DEBUG_VALUE("turnstile_s", turnstile_decider_output_.turnstile_s())
  JSON_DEBUG_VALUE("turnstile_stop_s", turnstile_decider_output_.turnstile_stop_s())
  JSON_DEBUG_VALUE("turnstile_target_lost_frame_count",
                   turnstile_decider_output_.target_turnstile_lost_frame_count())
  JSON_DEBUG_VALUE("turnstile_passable_status_stable_frame_count",
                   turnstile_decider_output_.turnstile_passable_status_stable_frame_count())
  JSON_DEBUG_VALUE("turnstile_cycle_closing_status_stable_frame_count",
                   turnstile_decider_output_.cycle_closing_status_stable_frame_count())
  JSON_DEBUG_VALUE("turnstile_reopen_open_status_continuous_frame_count",
                   turnstile_decider_output_.reopen_open_status_continuous_frame_count())
  JSON_DEBUG_VALUE("turnstile_release_by_open_timeout",
                   turnstile_decider_output_.release_by_open_timeout())
  JSON_DEBUG_VALUE("turnstile_closing_status_drop_consecutive_frame_count",
                   turnstile_decider_output_.closing_status_drop_consecutive_frame_count())
  JSON_DEBUG_VALUE("turnstile_closing_status_drop_emergency_active",
                   turnstile_decider_output_.closing_status_drop_emergency_active())
  JSON_DEBUG_VALUE("turnstile_emergency_opening_status_stable_frame_count",
                   turnstile_decider_output_.emergency_opening_status_stable_frame_count())
  SaveToSession();
  return true;
}

bool TurnstileLongitudinalDecider::UpdateCurrentReferencePath() {
  const auto& current_lane =
      session_->environmental_model().get_virtual_lane_manager()->get_current_lane();
  if (current_lane == nullptr) {
    return false;
  }
  reference_path_ = current_lane->get_reference_path();
  return reference_path_ != nullptr;
}

void TurnstileLongitudinalDecider::ResetCurrentFrameState() {
  target_turnstile_frenet_obs_ = nullptr;
  target_turnstile_obs_ = nullptr;
  front_vehicle_frenet_obs_ = nullptr;
  turnstile_decider_output_.set_front_car_id(agent::AgentDefaultInfo::kNoAgentId);
  turnstile_decider_output_.set_is_head_car(true);
  turnstile_decider_output_.set_has_target_turnstile(false);
  turnstile_decider_output_.set_turnstile_scene_type(
      static_cast<int32_t>(reference_path_->get_turnstile_scene_type()));
  turnstile_decider_output_.set_target_turnstile_obs_id(
      reference_path_->get_target_turnstile_obs_id());
  turnstile_decider_output_.set_side_turnstile_obs_id(
      reference_path_->get_side_turnstile_obs_id());
  turnstile_decider_output_.set_front_car_passed_in_current_cycle(
      front_car_passed_in_current_cycle_);
  turnstile_decider_output_.set_wait_reopen_required(wait_reopen_required_);
  turnstile_decider_output_.set_has_seen_gate_closed_status_after_front_car_pass(
      has_seen_gate_closed_status_after_front_car_pass_);
  turnstile_decider_output_.set_turnstile_passable_status_stable_frame_count(
      turnstile_passable_status_stable_frame_count_);
  turnstile_decider_output_.set_target_turnstile_lost_frame_count(
      target_turnstile_lost_frame_count_);
  turnstile_decider_output_.set_cycle_closing_status_stable_frame_count(
      cycle_closing_status_stable_frame_count_);
  turnstile_decider_output_.set_reopen_open_status_continuous_frame_count(
      reopen_open_status_continuous_frame_count_);
  turnstile_decider_output_.set_release_by_open_timeout(
      release_by_open_timeout_);
  turnstile_decider_output_.set_closing_status_drop_consecutive_frame_count(
      closing_status_drop_consecutive_frame_count_);
  turnstile_decider_output_.set_closing_status_drop_emergency_active(
      closing_status_drop_emergency_active_);
  turnstile_decider_output_.set_emergency_opening_status_stable_frame_count(
      emergency_opening_status_stable_frame_count_);
}

const FrenetObstacle* TurnstileLongitudinalDecider::FindTargetTurnstileFrenetObstacle()
    const {
  if (reference_path_ == nullptr) {
    return nullptr;
  }
  const auto target_id = reference_path_->get_target_turnstile_obs_id();
  const auto& turnstile_map = reference_path_->get_turnstile_obstacles_map();
  auto target_turnstile_iter = turnstile_map.find(target_id);
  if (target_turnstile_iter == turnstile_map.end() ||
      target_turnstile_iter->second == nullptr) {
    return nullptr;
  }
  return target_turnstile_iter->second.get();
}

void TurnstileLongitudinalDecider::UpdateTargetTurnstile() {
  target_turnstile_frenet_obs_ = FindTargetTurnstileFrenetObstacle();
  if (target_turnstile_frenet_obs_ == nullptr ||
      target_turnstile_frenet_obs_->obstacle() == nullptr) {
    target_turnstile_lost_frame_count_++;
    turnstile_decider_output_.set_target_turnstile_lost_frame_count(target_turnstile_lost_frame_count_);
    return;
  }

  const double ego_s = reference_path_->get_frenet_ego_state().s();
  const auto& boundary = target_turnstile_frenet_obs_->frenet_obstacle_boundary();
  if (boundary.s_end < ego_s) {
    target_turnstile_lost_frame_count_++;
    turnstile_decider_output_.set_target_turnstile_lost_frame_count(target_turnstile_lost_frame_count_);
    return;
  }

  target_turnstile_obs_ = target_turnstile_frenet_obs_->obstacle();
  target_turnstile_lost_frame_count_ = 0;
  turnstile_decider_output_.set_target_turnstile_lost_frame_count(0);
  turnstile_decider_output_.set_has_target_turnstile(true);
  turnstile_decider_output_.set_turnstile_s(boundary.s_start);
  turnstile_decider_output_.set_last_valid_target_turnstile_obs_id(target_turnstile_obs_->id());
}

bool TurnstileLongitudinalDecider::IsVehicleObstacle(
    const FrenetObstacle& frenet_obs) const {
  const auto type = frenet_obs.type();
  return type == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
         type == iflyauto::ObjectType::OBJECT_TYPE_VAN ||
         type == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRUCK ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRAILER ||
         type == iflyauto::ObjectType::OBJECT_TYPE_PICKUP ||
         type == iflyauto::ObjectType::OBJECT_TYPE_SUV ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MPV ||
         type == iflyauto::ObjectType::OBJECT_TYPE_ENGINEERING_VEHICLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_SPECIAL_VEHICLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_BICYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_OCC_CAR ||
         type == iflyauto::ObjectType::OBJECT_TYPE_OCC_CYCLIST;
}


bool TurnstileLongitudinalDecider::IsFrontVehicleNearTurnstile(
    const FrenetObstacle& front_vehicle_frenet_obs) const {
  if (target_turnstile_frenet_obs_ == nullptr) {
    return false;
  }
  const auto& front_vehicle_boundary = front_vehicle_frenet_obs.frenet_obstacle_boundary();
  const auto& turnstile_boundary =
      target_turnstile_frenet_obs_->frenet_obstacle_boundary();
  return front_vehicle_boundary.s_start <= turnstile_boundary.s_end + lon_config_.turnstile_near_margin;
}

const FrenetObstacle* TurnstileLongitudinalDecider::FindNearestFrontVehicle() const {
  if (reference_path_ == nullptr) {
    return nullptr;
  }
  const double ego_s = reference_path_->get_frenet_ego_state().s();
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const auto target_id = turnstile_decider_output_.target_turnstile_obs_id();

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

  const auto& obstacles = reference_path_->get_obstacles();
  const FrenetObstacle* best_obs = nullptr;
  double best_s = std::numeric_limits<double>::max();
  for (const auto& obs_ptr : obstacles) {
    if (obs_ptr == nullptr || obs_ptr->obstacle() == nullptr) {
      continue;
    }
    if (obs_ptr->id() == target_id || obs_ptr->id() == kHppTurnstileVirtualAgentId) {
      continue;
    }
    if (!IsVehicleObstacle(*obs_ptr)) {
      continue;
    }
    const auto& boundary = obs_ptr->frenet_obstacle_boundary();
    if (boundary.s_end <= ego_s) {
      continue;
    }
    if (boundary.s_start - ego_s > lon_config_.turnstile_front_vehicle_max_distance) {
      continue;
    }
    if (boundary.l_end < ego_boundary.l_start ||
        boundary.l_start > ego_boundary.l_end) {
      continue;
    }
    if (!IsFrontVehicleNearTurnstile(*obs_ptr)) {
      continue;
    }
    if (preferred_front_vehicle_id != agent::AgentDefaultInfo::kNoAgentId &&
        obs_ptr->id() == preferred_front_vehicle_id) {
      return obs_ptr.get();
    }
    if (boundary.s_start < best_s) {
      best_s = boundary.s_start;
      best_obs = obs_ptr.get();
    }
  }
  return best_obs;
}

void TurnstileLongitudinalDecider::UpdateFrontVehicle() {
  front_vehicle_frenet_obs_ = FindNearestFrontVehicle();
  if (front_vehicle_frenet_obs_ == nullptr || front_vehicle_frenet_obs_->obstacle() == nullptr) {
    turnstile_decider_output_.set_is_head_car(true);
    turnstile_decider_output_.set_front_car_id(agent::AgentDefaultInfo::kNoAgentId);
    return;
  }
  turnstile_decider_output_.set_is_head_car(false);
  turnstile_decider_output_.set_front_car_id(front_vehicle_frenet_obs_->id());
}

bool TurnstileLongitudinalDecider::IsTurnstileInClosingStatus(const Obstacle& turnstile_obs) const {
  return turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_UNKNOWN ||
         turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE;
}

bool TurnstileLongitudinalDecider::IsTurnstileInClosedStatus(
    const Obstacle& turnstile_obs) const {
  return turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_STATIC &&
         turnstile_obs.turnstile_open_ratio() <= lon_config_.turnstile_closed_status_threshold;
}

bool TurnstileLongitudinalDecider::IsTurnstileInCycleClosingStatus(
    const Obstacle& turnstile_obs) const {
  if (turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_UNKNOWN ||
      turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE) {
    return true;
  }
  return turnstile_obs.turnstile_open_ratio() <=
         lon_config_.turnstile_cycle_closing_status_ratio_threshold;
}

bool TurnstileLongitudinalDecider::IsTurnstileDroppedForEmergency(
    const Obstacle& turnstile_obs) const {
  if (turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_UNKNOWN ||
      turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE) {
    return true;
  }
  return turnstile_obs.turnstile_open_ratio() <=
         lon_config_.turnstile_closing_status_drop_ratio_threshold;
}

bool TurnstileLongitudinalDecider::IsTurnstileInOpeningStatus(const Obstacle& turnstile_obs) const {
  return turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_OPEN;
}

double TurnstileLongitudinalDecider::GetEffectiveTurnstileDtSec() const {
  if (lon_config_.turnstile_frame_dt_override_sec > 1e-6) {
    return lon_config_.turnstile_frame_dt_override_sec;
  }
  return std::max(lon_config_.delta_time, 1e-3);
}

bool TurnstileLongitudinalDecider::IsTurnstileInPassableStatus(const Obstacle& turnstile_obs) const {
  if (turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_UNKNOWN ||
      turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_CLOSE) {
    return false;
  }
  if (turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_STATIC) {
    return turnstile_obs.turnstile_open_ratio() >= lon_config_.turnstile_open_status_threshold;
  }
  if (turnstile_obs.turnstile_status() == iflyauto::GateBarrierStatus::MOTION_DIR_OPEN) {
    return turnstile_obs.turnstile_open_ratio() >= lon_config_.turnstile_passable_status_threshold;
  }
  return false;
}

void TurnstileLongitudinalDecider::UpdateTurnstilePassability() {
  if (target_turnstile_obs_ == nullptr) {
    turnstile_passable_status_stable_frame_count_ = 0;
    turnstile_decider_output_.set_turnstile_passable_status(false);
    turnstile_decider_output_.set_turnstile_passable_status_stable_frame_count(0);
    return;
  }
  const bool is_turnstile_passable_status = IsTurnstileInPassableStatus(*target_turnstile_obs_);
  if (is_turnstile_passable_status) {
    ++turnstile_passable_status_stable_frame_count_;
  } else {
    turnstile_passable_status_stable_frame_count_ = 0;
  }
  turnstile_decider_output_.set_turnstile_passable_status(is_turnstile_passable_status);
  turnstile_decider_output_.set_turnstile_passable_status_stable_frame_count(
      turnstile_passable_status_stable_frame_count_);
}

bool TurnstileLongitudinalDecider::IsFrontVehicleInPassingWindow(
    const FrenetObstacle& front_vehicle_frenet_obs) const {
  if (target_turnstile_frenet_obs_ == nullptr) {
    return false;
  }
  const auto& front_vehicle_boundary = front_vehicle_frenet_obs.frenet_obstacle_boundary();
  const auto& turnstile_boundary =
      target_turnstile_frenet_obs_->frenet_obstacle_boundary();
  return front_vehicle_boundary.s_end >= turnstile_boundary.s_start - lon_config_.turnstile_passing_window &&
         front_vehicle_boundary.s_start <= turnstile_boundary.s_end + lon_config_.turnstile_passing_window;
}

void TurnstileLongitudinalDecider::UpdateTurnstileCycleState() {
  if (wait_reopen_required_ && target_turnstile_obs_ != nullptr &&
      lon_config_.enable_turnstile_cycle_closing_status_reopen_release) {
    if (IsTurnstileInCycleClosingStatus(*target_turnstile_obs_)) {
      ++cycle_closing_status_stable_frame_count_;
    } else {
      cycle_closing_status_stable_frame_count_ = 0;
    }
    if (cycle_closing_status_stable_frame_count_ >=
        lon_config_.turnstile_cycle_closing_status_stable_frame_threshold) {
      has_seen_gate_closed_status_after_front_car_pass_ = true;
    }
  } else {
    cycle_closing_status_stable_frame_count_ = 0;
  }

  if (wait_reopen_required_ && target_turnstile_obs_ != nullptr &&
      lon_config_.enable_turnstile_open_timeout_release &&
      !has_seen_gate_closed_status_after_front_car_pass_) {
    if (turnstile_decider_output_.turnstile_passable_status()) {
      ++reopen_open_status_continuous_frame_count_;
    } else {
      reopen_open_status_continuous_frame_count_ = 0;
    }
    const double reopen_open_duration_sec =
        reopen_open_status_continuous_frame_count_ * GetEffectiveTurnstileDtSec();
    release_by_open_timeout_ =
        reopen_open_duration_sec >= lon_config_.turnstile_open_timeout_sec;
  } else {
    reopen_open_status_continuous_frame_count_ = 0;
    release_by_open_timeout_ = false;
  }

  bool has_current_front_vehicle = front_vehicle_frenet_obs_ != nullptr;
  if (!front_car_passed_in_current_cycle_ && had_valid_front_vehicle_in_prev_frame_ &&
      was_turnstile_passable_status_in_prev_frame_ && target_turnstile_frenet_obs_ != nullptr) {
    bool has_front_vehicle_passed = false;
    if (!has_current_front_vehicle ||
        (front_vehicle_frenet_obs_ != nullptr &&
         front_vehicle_frenet_obs_->id() != previous_front_vehicle_id_)) {
      has_front_vehicle_passed =
          previous_front_vehicle_s_ >=
          target_turnstile_frenet_obs_->frenet_obstacle_boundary().s_start -
              lon_config_.turnstile_passing_window;
    }
    if (has_front_vehicle_passed) {
      front_car_passed_in_current_cycle_ = true;
      wait_reopen_required_ = true;
      has_seen_gate_closed_status_after_front_car_pass_ = false;
      cycle_closing_status_stable_frame_count_ = 0;
      reopen_open_status_continuous_frame_count_ = 0;
      release_by_open_timeout_ = false;
    }
  }

  if (!wait_reopen_required_) {
    cycle_closing_status_stable_frame_count_ = 0;
    reopen_open_status_continuous_frame_count_ = 0;
    release_by_open_timeout_ = false;
  }

  if (lon_config_.enable_turnstile_closing_status_drop_emergency_stop &&
      target_turnstile_obs_ != nullptr &&
      (stage_ == TurnstileStage::PASSING ||
       stage_ == TurnstileStage::EMERGENCY_BLOCK ||
       stage_ == TurnstileStage::PASSABLE_RELEASE)) {
    if (IsTurnstileDroppedForEmergency(*target_turnstile_obs_)) {
      ++closing_status_drop_consecutive_frame_count_;
    } else {
      closing_status_drop_consecutive_frame_count_ = 0;
    }
    closing_status_drop_emergency_active_ =
        closing_status_drop_consecutive_frame_count_ >=
        lon_config_.turnstile_closing_status_drop_consecutive_frame_threshold;
  } else {
    closing_status_drop_consecutive_frame_count_ = 0;
    closing_status_drop_emergency_active_ = false;
  }

  if (stage_ == TurnstileStage::EMERGENCY_BLOCK &&
      target_turnstile_obs_ != nullptr &&
      IsTurnstileInOpeningStatus(*target_turnstile_obs_)) {
    ++emergency_opening_status_stable_frame_count_;
  } else {
    emergency_opening_status_stable_frame_count_ = 0;
  }

  if (front_vehicle_frenet_obs_ != nullptr) {
    previous_front_vehicle_id_ = front_vehicle_frenet_obs_->id();
    previous_front_vehicle_s_ = front_vehicle_frenet_obs_->frenet_obstacle_boundary().s_end;
    had_valid_front_vehicle_in_prev_frame_ = true;
  } else {
    had_valid_front_vehicle_in_prev_frame_ = false;
    previous_front_vehicle_id_ = agent::AgentDefaultInfo::kNoAgentId;
    previous_front_vehicle_s_ = 0.0;
  }
  was_turnstile_passable_status_in_prev_frame_ = turnstile_decider_output_.turnstile_passable_status();

  turnstile_decider_output_.set_front_car_passed_in_current_cycle(front_car_passed_in_current_cycle_);
  turnstile_decider_output_.set_wait_reopen_required(wait_reopen_required_);
  turnstile_decider_output_.set_has_seen_gate_closed_status_after_front_car_pass(
      has_seen_gate_closed_status_after_front_car_pass_);
  turnstile_decider_output_.set_cycle_closing_status_stable_frame_count(
      cycle_closing_status_stable_frame_count_);
  turnstile_decider_output_.set_reopen_open_status_continuous_frame_count(
      reopen_open_status_continuous_frame_count_);
  turnstile_decider_output_.set_release_by_open_timeout(release_by_open_timeout_);
  turnstile_decider_output_.set_closing_status_drop_consecutive_frame_count(
      closing_status_drop_consecutive_frame_count_);
  turnstile_decider_output_.set_closing_status_drop_emergency_active(
      closing_status_drop_emergency_active_);
  turnstile_decider_output_.set_emergency_opening_status_stable_frame_count(
      emergency_opening_status_stable_frame_count_);
}

bool TurnstileLongitudinalDecider::HasCompletedReopenCycle() const {
  if (!wait_reopen_required_) {
    return false;
  }
  const bool completed_by_cycle =
      lon_config_.enable_turnstile_cycle_closing_status_reopen_release &&
      has_seen_gate_closed_status_after_front_car_pass_ &&
      turnstile_passable_status_stable_frame_count_ >=
          lon_config_.turnstile_passable_status_stable_frame_threshold;
  const bool completed_by_timeout =
      lon_config_.enable_turnstile_open_timeout_release && release_by_open_timeout_;
  return completed_by_cycle || completed_by_timeout;
}

double TurnstileLongitudinalDecider::ComputeTurnstileStopS(
    const FrenetObstacle& turnstile_obs) const {
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const double turnstile_s = turnstile_obs.frenet_obstacle_boundary().s_start;
  return std::max(
      turnstile_s,
      ego_boundary.s_end + lon_config_.turnstile_min_forward_stop_buffer);
}

TurnstileLongitudinalDecider::TurnstileEventFlags
TurnstileLongitudinalDecider::BuildTurnstileEventFlags() const {
  TurnstileLongitudinalDecider::TurnstileEventFlags flags;
  flags.has_target_turnstile = turnstile_decider_output_.has_target_turnstile();
  flags.target_lost_timeout =
      target_turnstile_lost_frame_count_ >= lon_config_.turnstile_target_lost_tolerance_frames;
  flags.is_head_car = turnstile_decider_output_.is_head_car();
  flags.turnstile_passable_status = turnstile_decider_output_.turnstile_passable_status();
  flags.passable_status_stable =
      flags.turnstile_passable_status &&
      turnstile_passable_status_stable_frame_count_ >=
          lon_config_.turnstile_passable_status_stable_frame_threshold;
  flags.wait_reopen_required = wait_reopen_required_;
  flags.has_seen_gate_closed_status_after_front_car_pass =
      has_seen_gate_closed_status_after_front_car_pass_;
  flags.reopen_completed = HasCompletedReopenCycle();
  const double reopen_open_duration_sec =
      reopen_open_status_continuous_frame_count_ * GetEffectiveTurnstileDtSec();
  flags.reopen_timeout_halfway =
      lon_config_.enable_turnstile_open_timeout_release &&
      reopen_open_duration_sec >= 0.5 * lon_config_.turnstile_open_timeout_sec;
  flags.emergency_active = closing_status_drop_emergency_active_;
  flags.emergency_opening_status_stable =
      emergency_opening_status_stable_frame_count_ >=
      lon_config_.turnstile_emergency_opening_status_stable_frame_threshold;

  if (reference_path_ != nullptr && flags.has_target_turnstile) {
    const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
    const double turnstile_s = turnstile_decider_output_.turnstile_s();
    flags.ego_in_gate = ego_boundary.s_end >= turnstile_s;
    flags.ego_passed =
        ego_boundary.s_start > turnstile_s + lon_config_.turnstile_passed_clear_distance;
  }

  if (target_turnstile_obs_ != nullptr) {
    flags.gate_opening_status = IsTurnstileInOpeningStatus(*target_turnstile_obs_);
    flags.gate_closed_status = IsTurnstileInClosedStatus(*target_turnstile_obs_);
    flags.gate_closing_status = IsTurnstileInClosingStatus(*target_turnstile_obs_);
  }

  return flags;
}

TurnstileLongitudinalDecider::TurnstileStage
TurnstileLongitudinalDecider::ResolveWaitReopenStage(
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (flags.reopen_completed) {
    return TurnstileStage::PASSABLE_RELEASE;
  }
  if (!flags.is_head_car) {
    if (flags.reopen_timeout_halfway) {
      return TurnstileStage::FOLLOW_WAIT_OPEN_TIMEOUT;
    }
    return TurnstileStage::FOLLOW_WAIT_GATE_CLOSE;
  }
  if (!flags.has_seen_gate_closed_status_after_front_car_pass) {
    if (flags.gate_closing_status) {
      return TurnstileStage::HEAD_WAIT_CLOSED;
    }
    if (flags.reopen_timeout_halfway) {
      return TurnstileStage::FOLLOW_WAIT_OPEN_TIMEOUT;
    }
    return TurnstileStage::FOLLOW_WAIT_GATE_CLOSE;
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
    const TurnstileLongitudinalDecider::TurnstileEventFlags& flags) const {
  if (!flags.has_target_turnstile) {
    return flags.target_lost_timeout ? TurnstileStage::IDLE : stage_;
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
  if (stage_ == TurnstileStage::EMERGENCY_BLOCK) {
    return ResolveEmergencyExitStage(flags);
  }
  if (flags.wait_reopen_required) {
    return ResolveWaitReopenStage(flags);
  }
  if (!flags.is_head_car) {
    return TurnstileStage::FOLLOW_WAIT;
  }
  return ResolveHeadCarStage(flags);
}

void TurnstileLongitudinalDecider::ResetReopenCycleFlags() {
  front_car_passed_in_current_cycle_ = false;
  wait_reopen_required_ = false;
  has_seen_gate_closed_status_after_front_car_pass_ = false;
  cycle_closing_status_stable_frame_count_ = 0;
  reopen_open_status_continuous_frame_count_ = 0;
  release_by_open_timeout_ = false;
}

void TurnstileLongitudinalDecider::ResetEmergencyFlags() {
  closing_status_drop_consecutive_frame_count_ = 0;
  emergency_opening_status_stable_frame_count_ = 0;
  closing_status_drop_emergency_active_ = false;
}

void TurnstileLongitudinalDecider::OnEnterStage(TurnstileStage prev_stage,
                                                TurnstileStage new_stage) {
  if (prev_stage == new_stage) {
    return;
  }
  if (new_stage == TurnstileStage::IDLE || new_stage == TurnstileStage::PASSED) {
    ResetReopenCycleFlags();
    ResetEmergencyFlags();
    return;
  }
  if (new_stage == TurnstileStage::PASSABLE_RELEASE && wait_reopen_required_) {
    ResetReopenCycleFlags();
  }
  if (prev_stage == TurnstileStage::EMERGENCY_BLOCK &&
      new_stage != TurnstileStage::EMERGENCY_BLOCK) {
    ResetEmergencyFlags();
  }
}

void TurnstileLongitudinalDecider::UpdateTurnstileStage() {
  prev_stage_ = stage_;
  const TurnstileLongitudinalDecider::TurnstileEventFlags flags = BuildTurnstileEventFlags();
  const TurnstileStage new_stage = ResolveNextStage(flags);
  OnEnterStage(stage_, new_stage);
  stage_ = new_stage;

  turnstile_decider_output_.set_front_car_passed_in_current_cycle(
      front_car_passed_in_current_cycle_);
  turnstile_decider_output_.set_wait_reopen_required(wait_reopen_required_);
  turnstile_decider_output_.set_has_seen_gate_closed_status_after_front_car_pass(
      has_seen_gate_closed_status_after_front_car_pass_);
  turnstile_decider_output_.set_release_by_open_timeout(release_by_open_timeout_);
  turnstile_decider_output_.set_closing_status_drop_emergency_active(
      closing_status_drop_emergency_active_);
  turnstile_decider_output_.set_emergency_opening_status_stable_frame_count(
      emergency_opening_status_stable_frame_count_);
  turnstile_decider_output_.set_turnstile_stage(static_cast<int32_t>(stage_));
  if (target_turnstile_frenet_obs_ != nullptr) {
    turnstile_decider_output_.set_turnstile_stop_s(
        ComputeTurnstileStopS(*target_turnstile_frenet_obs_));
  }
}

bool TurnstileLongitudinalDecider::ShouldCreateVirtualObstacle() const {
  if (!turnstile_decider_output_.has_target_turnstile()) {
    return false;
  }
  if (stage_ == TurnstileStage::EMERGENCY_BLOCK) {
    return true;
  }
  return stage_ != TurnstileStage::IDLE &&
         stage_ != TurnstileStage::PASSABLE_RELEASE &&
         stage_ != TurnstileStage::PASSING &&
         stage_ != TurnstileStage::PASSED;
}

bool TurnstileLongitudinalDecider::AddVirtualObstacle() {
  if (reference_path_ == nullptr || target_turnstile_frenet_obs_ == nullptr) {
    return false;
  }
  ReferencePathPoint ref_point;
  const auto& ego_boundary = reference_path_->get_ego_frenet_boundary();
  const double turnstile_s = target_turnstile_frenet_obs_->frenet_obstacle_boundary().s_start;
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

void TurnstileLongitudinalDecider::SaveToSession() {
  auto& session_output =
      session_->mutable_planning_context()->mutable_turnstile_longitudinal_decider_output();
  session_output = turnstile_decider_output_;
}

}  // namespace planning
