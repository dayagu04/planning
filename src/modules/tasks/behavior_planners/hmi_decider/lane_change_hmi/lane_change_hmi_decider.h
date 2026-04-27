#pragma once

#include "common/utils/hysteresis_decision.h"
#include "modules/context/planning_context.h"

namespace planning {

class LaneChangeHmiDecider {
 public:
  LaneChangeHmiDecider() = default;

  explicit LaneChangeHmiDecider(framework::Session* session);
  ~LaneChangeHmiDecider() = default;

  bool Execute();

 private:
  void UpdateHMIInfo();
  void UpdateTurnSignal();
  bool IsDistanceToOriginLineEnough(RampDirection ramp_direction);
  iflyauto::LandingPoint CalculateLandingPoint(
      bool is_lane_keeping,
      const LaneChangeDeciderOutput& lane_change_decider_output);

 private:
  framework::Session* session_ = nullptr;
  RampDirection last_frame_dir_turn_signal_road_to_ramp_ = RAMP_NONE;
  int lc_state_complete_frame_nums_ = 31;
  HysteresisDecision lane_borrow_turn_signal_speed_hysteresis_{0.5, 0.3};
  HysteresisDecision lane_borrow_boundary_dist_hysteresis_{-0.6, -0.8};
};

}  // namespace planning