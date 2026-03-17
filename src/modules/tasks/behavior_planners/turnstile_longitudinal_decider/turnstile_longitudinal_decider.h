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
    FOLLOW_CYCLE_CONSUMED_WAIT_CLOSE = 3,
    HEAD_WAIT_CLOSED = 4,
    HEAD_WAIT_REOPEN = 5,
    HEAD_WAIT_OPENING = 6,
    PASSABLE_RELEASE = 7,
    PASSING = 8,
    PASSED = 9,
  };

  TurnstileLongitudinalDecider(const EgoPlanningConfigBuilder* config_builder,
                               framework::Session* session);
  ~TurnstileLongitudinalDecider() override = default;

  bool Execute() override;

 private:
  bool UpdateCurrentReferencePath();
  void UpdateTargetTurnstile();
  void UpdateFrontVehicle();
  void UpdateTurnstilePassability();
  void UpdateTurnstileCycleState();
  void UpdateTurnstileStage();
  bool ShouldCreateVirtualObstacle() const;
  bool AddVirtualObstacle();
  void SaveToSession();
  void ResetCurrentFrameState();
  const FrenetObstacle* FindTargetTurnstileFrenetObstacle() const;
  const FrenetObstacle* FindNearestFrontVehicle() const;
  bool IsVehicleObstacle(const FrenetObstacle& frenet_obs) const;
  bool IsFrontVehicleNearTurnstile(const FrenetObstacle& front_vehicle_frenet_obs) const;
  bool IsFrontVehicleInPassingWindow(const FrenetObstacle& front_vehicle_frenet_obs) const;
  bool IsTurnstileCurrentlyPassable(const Obstacle& turnstile_obs) const;
  bool IsTurnstileClosedLike(const Obstacle& turnstile_obs) const;
  bool IsTurnstileOpeningLike(const Obstacle& turnstile_obs) const;
  bool HasCompletedReopenCycle() const;
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
  bool was_turnstile_passable_in_prev_frame_ = false;

  int32_t target_turnstile_lost_frame_count_ = 0;
  int32_t turnstile_passable_stable_frame_count_ = 0;
  bool front_car_passed_in_current_cycle_ = false;
  bool wait_reopen_required_ = false;
  bool has_seen_gate_closed_after_front_car_pass_ = false;
};

}  // namespace planning
