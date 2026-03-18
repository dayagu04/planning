#pragma once

#include <cstdint>
#include <memory>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "reference_path.h"
#include "tasks/task.h"
#include "turnstile_longitudinal_decider_output.h"

namespace planning {

class TurnstileLongitudinalDecider : public Task {
 public:
  enum class TurnstileStage {
    IDLE = 0,
    APPROACHING = 1,
    FOLLOW_WAIT = 2,
    FOLLOW_WAIT_GATE_CLOSE = 3,
    HEAD_WAIT_CLOSED = 4,
    HEAD_WAIT_REOPEN = 5,
    HEAD_WAIT_FULLY_OPEN = 6,
    PASSABLE_RELEASE = 7,
    PASSING = 8,
    PASSED = 9,
    FOLLOW_WAIT_OPEN_TIMEOUT = 10,
    EMERGENCY_BLOCK = 11,
  };

  TurnstileLongitudinalDecider(const EgoPlanningConfigBuilder* config_builder,
                               framework::Session* session);
  ~TurnstileLongitudinalDecider() override = default;

  bool Execute() override;

 private:
  struct TurnstileEventFlags {
    bool has_target_turnstile = false;
    bool target_lost_timeout = false;
    bool ego_passed = false;
    bool ego_in_gate = false;
    bool is_head_car = true;
    bool turnstile_passable_status = false;
    bool passable_status_stable = false;
    bool gate_opening_status = false;
    bool gate_closed_status = false;
    bool gate_closing_status = false;
    bool wait_reopen_required = false;
    bool has_seen_gate_closed_status_after_front_car_pass = false;
    bool reopen_completed = false;
    bool reopen_timeout_halfway = false;
    bool emergency_active = false;
    bool emergency_opening_status_stable = false;
  };

  bool UpdateCurrentReferencePath();
  void UpdateTargetTurnstile();
  void UpdateFrontVehicle();
  void UpdateTurnstilePassability();
  void UpdateTurnstileCycleState();
  void UpdateTurnstileStage();
  TurnstileEventFlags BuildTurnstileEventFlags() const;
  TurnstileStage ResolveWaitReopenStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveHeadCarStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveEmergencyExitStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveNextStage(const TurnstileEventFlags& flags) const;
  void OnEnterStage(TurnstileStage prev_stage, TurnstileStage new_stage);
  void ResetReopenCycleFlags();
  void ResetEmergencyFlags();
  bool ShouldCreateVirtualObstacle() const;
  bool AddVirtualObstacle();
  void SaveToSession();
  void ResetCurrentFrameState();
  const FrenetObstacle* FindTargetTurnstileFrenetObstacle() const;
  const FrenetObstacle* FindNearestFrontVehicle() const;
  bool IsVehicleObstacle(const FrenetObstacle& frenet_obs) const;
  bool IsFrontVehicleNearTurnstile(const FrenetObstacle& front_vehicle_frenet_obs) const;
  bool IsFrontVehicleInPassingWindow(const FrenetObstacle& front_vehicle_frenet_obs) const;
  bool IsTurnstileInPassableStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInClosingStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInClosedStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInCycleClosingStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileDroppedForEmergency(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInOpeningStatus(const Obstacle& turnstile_obs) const;
  bool HasCompletedReopenCycle() const;
  double GetEffectiveTurnstileDtSec() const;
  double ComputeTurnstileStopS(const FrenetObstacle& turnstile_obs) const;

 private:
  const LongitudinalDeciderV3Config lon_config_;
  std::shared_ptr<ReferencePath> reference_path_;

  TurnstileLongitudinalDeciderOutput turnstile_decider_output_;
  TurnstileStage stage_ = TurnstileStage::IDLE;
  TurnstileStage prev_stage_ = TurnstileStage::IDLE;

  const FrenetObstacle* target_turnstile_frenet_obs_ = nullptr;
  const Obstacle* target_turnstile_obs_ = nullptr;
  const FrenetObstacle* front_vehicle_frenet_obs_ = nullptr;

  int32_t previous_front_vehicle_id_ = agent::AgentDefaultInfo::kNoAgentId;
  double previous_front_vehicle_s_ = 0.0;
  bool had_valid_front_vehicle_in_prev_frame_ = false;
  bool was_turnstile_passable_status_in_prev_frame_ = false;

  int32_t target_turnstile_lost_frame_count_ = 0;
  int32_t turnstile_passable_status_stable_frame_count_ = 0;
  int32_t cycle_closing_status_stable_frame_count_ = 0;
  int32_t reopen_open_status_continuous_frame_count_ = 0;
  int32_t closing_status_drop_consecutive_frame_count_ = 0;
  int32_t emergency_opening_status_stable_frame_count_ = 0;
  bool front_car_passed_in_current_cycle_ = false;
  bool wait_reopen_required_ = false;
  bool has_seen_gate_closed_status_after_front_car_pass_ = false;
  bool release_by_open_timeout_ = false;
  bool closing_status_drop_emergency_active_ = false;
};

}  // namespace planning
