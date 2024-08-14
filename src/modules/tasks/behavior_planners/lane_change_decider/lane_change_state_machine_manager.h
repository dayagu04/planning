#pragma once

#include <unordered_map>
#include "behavior_planners/lane_change_decider/lane_change_request_manager.h"
#include "session.h"
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

enum RelativeDirection {
  ON_LEFT = 0,
  ON_RIGHT = 1,
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
  std::pair<int, int> gap;
  bool gap_valid{false};
  bool gap_approached{false};
  bool gap_insertable{false};
  bool side_approach{false};
  bool should_suspend{false};

  // lc valid related
  bool lc_pause{false};
  int lc_pause_id{-1000};
  bool should_premove{false};
  std::string lc_invalid_reason{"none"};

  // lc back related
  double tr_pause_dv{0.0};
  double tr_pause_l{0.0};
  double tr_pause_s{-100.0};
  bool accident_back{false};

  // clear lc related
  bool need_clear_lb_car{false};

  // not related but needed
  bool accident_ahead{false};
  bool close_to_accident{false};

  // back to state machine only for debug
  bool lc_should_back{false};
  bool lc_valid{false};
  std::string lc_back_reason{"none"};
  void Reset() {
    gap_valid = false;
    gap_approached = false;
    gap_insertable = false;
    side_approach = false;
    should_suspend = false;
    lc_pause = false;
    lc_pause_id = -1000;
    should_premove = false;
    lc_invalid_reason = "none";
    tr_pause_dv = 0.0;
    tr_pause_l = 0.0;
    tr_pause_s = -100.0;
    accident_back = false;
    need_clear_lb_car = false;
    accident_ahead = false;
    close_to_accident = false;
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
                                RequestSource* const lane_change_type);
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
  bool CheckIfCompleteToLaneKeeping();
  bool CheckIfInPerfectLaneKeeping();
  bool CheckIfCompleteToCancel();
  bool CheckIfCancelToLaneKeeping();
  bool CheckIfCancelTimeOut();

  void LaneChangeInfoReset();

  void CheckLaneChangeValid(RequestType direction);
  LaneChangeStageInfo CheckLCGapFeasible(RequestType direction);
  void CheckLaneChangeBackValid(RequestType direction);
  LaneChangeStageInfo CheckIfNeedLCBack(RequestType direction);

  void MakeFixLane();
  void UpdateStateMachine();
  void GenerateStateMachineOutput();
  void CalculateSideGapFeasible(
      const std::vector<TrackedObject>& vec_side_obstacle,
      LaneChangeStageInfo* const lc_state_info);
  void CalculateFrontGapFeasible(
      const std::vector<TrackedObject>& vec_side_obstacle,
      LaneChangeStageInfo* const lc_state_info);
  void CalculateSideAreaIfNeedBack(
      const std::vector<TrackedObject>& vec_side_obstacle,
      const RequestType& direction, LaneChangeStageInfo* const lc_state_info);
  void CalculateFrontAreaIfNeedBack(
      const std::vector<TrackedObject>& vec_side_obstacle,
      const RequestType& direction, LaneChangeStageInfo* const lc_state_info);
  bool TimeOut(const bool& trigger, bool* is_start_count, double* time_count,
               const double& threshold);
  void UpdateCoarsePlanningInfo();
  void UpdateAdInfo();
  void UpdateStateMachineDebugInfo();
  void GenerateTurnSignalForSplitCase();
  bool IsSplitCase(RampDirection* ramp_direction);
  bool MakeFrontDiff(double* lat_diff,
                     const std::shared_ptr<ReferencePath> reference_path);
  bool IsOffTurnLight(const RampDirection ramp_direction);
  vector<Point2D> CarSideCorners();

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

  std::vector<TrackInfo> near_cars_target_;
  std::vector<TrackInfo> near_cars_origin_;
  TrackInfo lc_invalid_track_;
  TrackInfo lc_back_track_;
  bool behavior_suspend_ = false;  // lateral suspend
  std::vector<int> suspend_obs_;   // lateral suspend obstacles
  double start_move_dist_lane_ = 0;
  bool must_change_lane_ = false;
  int scenario_ = SCENARIO_CRUISE;
  RampDirection road_to_ramp_turn_signal_ = RAMP_NONE;
};
}  // namespace planning