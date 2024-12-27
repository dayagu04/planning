#pragma once

#include <cstdint>
#include <unordered_map>
#include "behavior_planners/lane_change_decider/lane_change_request_manager.h"
#include "define/geometry.h"
#include "session.h"
#include "virtual_lane.h"
namespace planning {
struct StateTransitionInfo {
  StateMachineLaneChangeStatus lane_change_status = kLaneKeeping;
  RequestType lane_change_direction = NO_CHANGE;
  RequestSource lane_change_type = NO_REQUEST;
  void Rest() {
    lane_change_status = kLaneKeeping;
    lane_change_direction = NO_CHANGE;
    lane_change_type = NO_REQUEST;
  }
};

struct LaneChangeTimer {
  bool propose_time_count_ = false;
  double propose_at_time_ = 0.0;
  bool execution_time_count_ = false;
  double execution_at_time_ = 0.0;
  bool gap_not_available_time_count_ = false;
  double gap_not_available_at_time_ = 0.0;
  bool hold_time_count_ = false;
  double hold_at_time_ = 0.0;
  bool is_safe_hold_to_execution_count_ = false;
  double is_safe_hold_to_execution_at_time_ = 0.0;
  bool complete_time_count_ = false;
  double complete_at_time_ = 0.0;
  bool cancel_time_count_ = false;
  double cancel_at_time_ = 0.0;
  void Reset() {
    propose_time_count_ = false;
    propose_at_time_ = IflyTime::Now_ms();

    execution_time_count_ = false;
    execution_at_time_ = IflyTime::Now_ms();

    gap_not_available_time_count_ = false;
    gap_not_available_at_time_ = IflyTime::Now_ms();

    hold_time_count_ = false;
    hold_at_time_ = IflyTime::Now_ms();

    is_safe_hold_to_execution_count_ = false;
    is_safe_hold_to_execution_at_time_ = IflyTime::Now_ms();

    complete_time_count_ = false;
    complete_at_time_ = IflyTime::Now_ms();

    cancel_time_count_ = false;
    cancel_at_time_ = IflyTime::Now_ms();
  }
};

struct LaneChangeStageInfo {
  bool gap_insertable{false};
  // lc valid related
  bool should_premove{false};
  std::string lc_invalid_reason{"none"};
  // back to state machine only for debug
  bool lc_should_back{false};
  bool lc_valid{false};
  std::string lc_back_reason{"none"};
  void Reset() {
    should_premove = false;
    lc_invalid_reason = "none";
    lc_should_back = false;
    lc_valid = false;
    lc_back_reason = "none";
  }
};

class LaneChangeStateMachineManager {
 public:
  LaneChangeStateMachineManager(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session,
      std::shared_ptr<LaneChangeRequestManager> lane_change_req_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  void init();
  virtual ~LaneChangeStateMachineManager() = default;

  void Update();
  void ResetStateMachine();

 private:
  void RunStateMachine();
  bool CheckIfProposeLaneChange(RequestType* const lane_change_direction,
                                RequestSource* const lane_change_type) const;
  bool CheckIfProposeToExecution(const RequestType& lane_change_direction,
                                 const RequestSource& lane_change_type);
  bool CheckIfProposeToCancel(const RequestType& lane_change_direction,
                              const RequestSource& lane_change_type);
  bool CheckIfLaneChangeComplete(const RequestType& lane_change_direction,
                                 const RequestSource& lane_change_type);
  bool CheckIfExecutionToCancel(const RequestType& lane_change_direction,
                                const RequestSource& lane_change_type);
  bool CheckIfExecutionToHold(const RequestType& lane_change_direction,
                              const RequestSource& lane_change_type);
  bool CheckIfHoldToCancel(const RequestType& lane_change_direction,
                           const RequestSource& lane_change_type);
  bool CheckIfHoldToExecution(const RequestType& lane_change_direction,
                              const RequestSource& lane_change_type);
  bool CheckIfCompleteToLaneKeeping() const;
  bool CheckIfInPerfectLaneKeeping() const;
  bool CheckIfCancelToLaneKeeping() const;
  bool CheckIfCompleteToCancel();
  bool CheckIfCancelTimeOut();

  void LaneChangeInfoReset();

  void CheckLaneChangeValid(RequestType direction);
  LaneChangeStageInfo CheckLCGapFeasible(RequestType direction);
  void CheckLaneChangeBackValid(RequestType direction);
  LaneChangeStageInfo CheckIfNeedLCBack(RequestType direction);

  void MakeFixLane();
  void UpdateStateMachine();
  void GenerateStateMachineOutput();
  void CalculateSideGapFeasible(LaneChangeStageInfo* const lc_state_info);
  void CalculateFrontGapFeasible(LaneChangeStageInfo* const lc_state_info);
  void CalculateSideAreaIfNeedBack(LaneChangeStageInfo* const lc_state_info);
  void CalculateFrontAreaIfNeedBack(LaneChangeStageInfo* const lc_state_info);
  bool TimeOut(const bool& trigger, bool* is_start_count, double* time_count,
               const double& threshold);
  void UpdateCoarsePlanningInfo();
  void UpdateStateMachineDebugInfo();
  void GenerateTurnSignalForSplitRegion();
  bool IsSplitRegion(RampDirection* ramp_direction);
  void CalculateLatOffsetOfOverlappedLanes(
      double* lat_diff, const std::shared_ptr<ReferencePath> reference_path);
  bool IsOffTurnLight(const RampDirection ramp_direction);
  const double CalculateEgoFrontLineLength();

  iflyauto::LaneBoundaryType MakesureCurrentBoundaryType(
      const RequestType lc_request) const;

  void PreProcess();
  bool IsLargeAgent(const planning_data::DynamicAgentNode* agent);
  void CalculateLatCloseValue();

 private:
  ScenarioStateMachineConfig config_;
  framework::Session* session_;
  std::shared_ptr<LaneChangeRequestManager> lc_req_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lc_lane_mgr_;
  StateTransitionInfo transition_info_;
  LaneChangeTimer lc_timer_;
  LaneChangeStageInfo lane_change_stage_info_;
  double pre_ego_l_ = 0.0;
  double pre_lane_change_finish_time_ = 0.0;
  int lc_back_cnt_ = 0;
  int lc_valid_cnt_ = 0;
  RequestType map_turn_signal_ = NO_CHANGE;

  TrackInfo lc_invalid_track_;
  TrackInfo lc_back_track_;
  bool must_change_lane_ = false;
  int scenario_ = SCENARIO_CRUISE;
  RampDirection road_to_ramp_turn_signal_ = RAMP_NONE;
  double overlap_lane_virtual_id_ = 0;
  int propose_state_frame_nums_ = 0;
  int execution_state_frame_nums_ = 0;
  double lat_close_boundary_offset_ = 0;
  const planning_data::DynamicAgentNode* target_lane_front_node_ = nullptr;
  const planning_data::DynamicAgentNode* target_lane_middle_node_ = nullptr;
  const planning_data::DynamicAgentNode* target_lane_rear_node_ = nullptr;
  const planning_data::DynamicAgentNode* ego_lane_front_node_ = nullptr;
  bool is_large_car_in_side_ = false;
};
}  // namespace planning