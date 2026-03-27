#pragma once

#include <cstdint>

#include "agent/agent.h"

namespace planning {

class TurnstileLongitudinalDeciderOutput {
 public:
  void Clear() {
    has_target_turnstile_ = false;
    target_turnstile_obs_id_ = agent::AgentDefaultInfo::kNoAgentId;
    side_turnstile_obs_id_ = agent::AgentDefaultInfo::kNoAgentId;
    turnstile_scene_type_ = 0;
    is_head_car_ = true;
    front_car_id_ = agent::AgentDefaultInfo::kNoAgentId;
    front_car_passed_in_current_cycle_ = false;
    wait_reopen_required_ = false;
    has_seen_gate_closed_status_after_front_car_pass_ = false;
    turnstile_passable_status_ = false;
    turnstile_stage_ = 0;
    stop_required_ = false;
    stop_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
    turnstile_stop_s_ = 0.0;
    turnstile_s_ = 0.0;
    target_turnstile_lost_frame_count_ = 0;
    turnstile_passable_status_stable_frame_count_ = 0;
    cycle_closing_status_stable_frame_count_ = 0;
    reopen_open_status_continuous_frame_count_ = 0;
    release_by_open_timeout_ = false;
    closing_status_drop_consecutive_frame_count_ = 0;
    closing_status_drop_emergency_active_ = false;
    emergency_opening_status_stable_frame_count_ = 0;
    last_valid_target_turnstile_obs_id_ = agent::AgentDefaultInfo::kNoAgentId;
  }

  bool has_target_turnstile() const { return has_target_turnstile_; }
  void set_has_target_turnstile(const bool value) {
    has_target_turnstile_ = value;
  }

  int32_t target_turnstile_obs_id() const { return target_turnstile_obs_id_; }
  void set_target_turnstile_obs_id(const int32_t value) {
    target_turnstile_obs_id_ = value;
  }

  int32_t side_turnstile_obs_id() const { return side_turnstile_obs_id_; }
  void set_side_turnstile_obs_id(const int32_t value) {
    side_turnstile_obs_id_ = value;
  }

  int32_t turnstile_scene_type() const { return turnstile_scene_type_; }
  void set_turnstile_scene_type(const int32_t value) {
    turnstile_scene_type_ = value;
  }

  bool is_head_car() const { return is_head_car_; }
  void set_is_head_car(const bool value) { is_head_car_ = value; }

  int32_t front_car_id() const { return front_car_id_; }
  void set_front_car_id(const int32_t value) { front_car_id_ = value; }

  bool front_car_passed_in_current_cycle() const {
    return front_car_passed_in_current_cycle_;
  }
  void set_front_car_passed_in_current_cycle(const bool value) {
    front_car_passed_in_current_cycle_ = value;
  }

  bool wait_reopen_required() const { return wait_reopen_required_; }
  void set_wait_reopen_required(const bool value) {
    wait_reopen_required_ = value;
  }

  bool has_seen_gate_closed_status_after_front_car_pass() const {
    return has_seen_gate_closed_status_after_front_car_pass_;
  }
  void set_has_seen_gate_closed_status_after_front_car_pass(const bool value) {
    has_seen_gate_closed_status_after_front_car_pass_ = value;
  }

  bool turnstile_passable_status() const { return turnstile_passable_status_; }
  void set_turnstile_passable_status(const bool value) {
    turnstile_passable_status_ = value;
  }

  int32_t turnstile_stage() const { return turnstile_stage_; }
  void set_turnstile_stage(const int32_t value) { turnstile_stage_ = value; }

  bool stop_required() const { return stop_required_; }
  void set_stop_required(const bool value) { stop_required_ = value; }

  int32_t stop_virtual_agent_id() const { return stop_virtual_agent_id_; }
  void set_stop_virtual_agent_id(const int32_t value) {
    stop_virtual_agent_id_ = value;
  }

  double turnstile_stop_s() const { return turnstile_stop_s_; }
  void set_turnstile_stop_s(const double value) { turnstile_stop_s_ = value; }

  double turnstile_s() const { return turnstile_s_; }
  void set_turnstile_s(const double value) { turnstile_s_ = value; }

  int32_t target_turnstile_lost_frame_count() const {
    return target_turnstile_lost_frame_count_;
  }
  void set_target_turnstile_lost_frame_count(const int32_t value) {
    target_turnstile_lost_frame_count_ = value;
  }

  int32_t turnstile_passable_status_stable_frame_count() const {
    return turnstile_passable_status_stable_frame_count_;
  }
  void set_turnstile_passable_status_stable_frame_count(const int32_t value) {
    turnstile_passable_status_stable_frame_count_ = value;
  }

  int32_t cycle_closing_status_stable_frame_count() const {
    return cycle_closing_status_stable_frame_count_;
  }
  void set_cycle_closing_status_stable_frame_count(const int32_t value) {
    cycle_closing_status_stable_frame_count_ = value;
  }

  int32_t reopen_open_status_continuous_frame_count() const {
    return reopen_open_status_continuous_frame_count_;
  }
  void set_reopen_open_status_continuous_frame_count(const int32_t value) {
    reopen_open_status_continuous_frame_count_ = value;
  }

  bool release_by_open_timeout() const { return release_by_open_timeout_; }
  void set_release_by_open_timeout(const bool value) {
    release_by_open_timeout_ = value;
  }

  int32_t closing_status_drop_consecutive_frame_count() const {
    return closing_status_drop_consecutive_frame_count_;
  }
  void set_closing_status_drop_consecutive_frame_count(const int32_t value) {
    closing_status_drop_consecutive_frame_count_ = value;
  }

  bool closing_status_drop_emergency_active() const {
    return closing_status_drop_emergency_active_;
  }
  void set_closing_status_drop_emergency_active(const bool value) {
    closing_status_drop_emergency_active_ = value;
  }

  int32_t emergency_opening_status_stable_frame_count() const {
    return emergency_opening_status_stable_frame_count_;
  }
  void set_emergency_opening_status_stable_frame_count(const int32_t value) {
    emergency_opening_status_stable_frame_count_ = value;
  }

  int32_t last_valid_target_turnstile_obs_id() const {
    return last_valid_target_turnstile_obs_id_;
  }
  void set_last_valid_target_turnstile_obs_id(const int32_t value) {
    last_valid_target_turnstile_obs_id_ = value;
  }

 private:
  bool has_target_turnstile_ = false;
  int32_t target_turnstile_obs_id_ = agent::AgentDefaultInfo::kNoAgentId;
  int32_t side_turnstile_obs_id_ = agent::AgentDefaultInfo::kNoAgentId;
  int32_t turnstile_scene_type_ = 0;
  bool is_head_car_ = true;
  int32_t front_car_id_ = agent::AgentDefaultInfo::kNoAgentId;
  bool front_car_passed_in_current_cycle_ = false;
  bool wait_reopen_required_ = false;
  bool has_seen_gate_closed_status_after_front_car_pass_ = false;
  bool turnstile_passable_status_ = false;
  int32_t turnstile_stage_ = 0;
  bool stop_required_ = false;
  int32_t stop_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
  double turnstile_stop_s_ = 0.0;
  double turnstile_s_ = 0.0;
  int32_t target_turnstile_lost_frame_count_ = 0;
  int32_t turnstile_passable_status_stable_frame_count_ = 0;
  int32_t cycle_closing_status_stable_frame_count_ = 0;
  int32_t reopen_open_status_continuous_frame_count_ = 0;
  bool release_by_open_timeout_ = false;
  int32_t closing_status_drop_consecutive_frame_count_ = 0;
  bool closing_status_drop_emergency_active_ = false;
  int32_t emergency_opening_status_stable_frame_count_ = 0;
  int32_t last_valid_target_turnstile_obs_id_ =
      agent::AgentDefaultInfo::kNoAgentId;
};

}  // namespace planning
