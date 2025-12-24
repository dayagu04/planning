#pragma once
#include "common/task_basic_types.h"
#include "reference_path.h"
#include "session.h"
#include "utils/hysteresis_decision.h"

namespace planning {
constexpr int CONTROL_TIME = 30;
constexpr int COOLING_TIME = 40;
constexpr int END_DEBOUNCE_TIME = 3;
enum class EmergecyLevel { NONE, LOW, MEDIUM, HIGH };
enum class CancelNudgeReason { NONE, NO_OVERLAP, LARGE_LAT_DISTANCE, LANE_CHANGE, ST_UNION, OTHER };
struct NudgeInfo {
  NudgeInfo() {}
  NudgeInfo(uint32 i_id, NudgeDirection i_nudge_direction,
            double i_min_l_to_ref, double i_max_l_to_ref,
            EmergecyLevel i_emergency_level) {
    id = i_id;
    nudge_direction = i_nudge_direction;
    min_l_to_ref = i_min_l_to_ref;
    max_l_to_ref = i_max_l_to_ref;
    emergency_level = i_emergency_level;
  }
  void Reset() {
    id = 0;
    nudge_direction = NudgeDirection::NONE;
    min_l_to_ref = 0;
    max_l_to_ref = 0;
    emergency_level = EmergecyLevel::NONE;
  }

  uint32 id;
  NudgeDirection nudge_direction = NudgeDirection::NONE;
  double min_l_to_ref = 0;
  double max_l_to_ref = 0;
  EmergecyLevel emergency_level;
  CancelNudgeReason cancel_nudge_reason;
};


enum class SideNudgeState : int {
  IDLE,
  COOLING_DONW,
  CONTROL
};

class SideNudgeLateralOffsetDecider {
 public:
  SideNudgeLateralOffsetDecider() = default;
  SideNudgeLateralOffsetDecider(framework::Session *session, const EgoPlanningConfigBuilder* config_builder);
  ~SideNudgeLateralOffsetDecider() = default;
  bool Process();
  double lat_offset() const { return lateral_offset_; }
  const NudgeInfo& nudge_info() const { return nudge_info_; }
  void Reset();
  SideNudgeState nudge_state() const { return current_state_; }

 private:
  void RunStateMachine();
  bool IsStartNudge();
  bool LatOffsetCalculate();
  void UpdateNudgeInfo();
  bool IsStopNudge();
  bool IsStopNudgeDirectly();
  void UpdateCurrentState();
  void ResetIdleState();
  bool Init();
  double DesireLateralOffsetSideWay(double base_distance);
  void Log();
  framework::Session* session_;
  LateralOffsetDeciderConfig config_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;

  HysteresisDecision is_control_time_enough_{CONTROL_TIME, 1};
  HysteresisDecision is_control_{1, END_DEBOUNCE_TIME};
  HysteresisDecision is_coodown_time_enough_{COOLING_TIME, 1};

  SideNudgeState current_state_ = SideNudgeState::IDLE;
  NudgeInfo nudge_info_;
  double lateral_offset_ = 0.0;
};
}  // namespace planning