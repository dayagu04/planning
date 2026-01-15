#pragma once

#include "lane_change_request.h"
#include "tracked_object.h"
#include "utils/hysteresis_decision.h"
#include "modules/tasks/task_interface/potential_dangerous_agent_decider_output.h"


namespace planning {

/// @brief 动态障碍物紧急避障换道请求
class DynamicAgentEmergenceAvoidRequest : public LaneChangeRequest {
 public:
  DynamicAgentEmergenceAvoidRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~DynamicAgentEmergenceAvoidRequest() = default;
  void Init();
  void Update(int lc_status);

  void Reset();
  virtual void SetLaneChangeCmd(std::uint8_t lane_change_cmd) {
    lane_change_cmd_ = lane_change_cmd;
  }
  virtual void SetLaneChangeCancelFromTrigger(bool trigger_lane_change_cancel) {
    trigger_lane_change_cancel_ = trigger_lane_change_cancel;
  }
  virtual IntCancelReasonType lc_request_cancel_reason() {
    return lc_request_cancel_reason_;
  }

 private:
  void UpdateDynamicAgentEmergencyAvoidanceSituation();

  void GenerateLaneChangeDirection();

  void CheckLaneChangeDirection(int lc_status);

  bool CheckEmergencyDynamicAgent();

  bool CheckEmergencyBaseLastEmergencyAvoid();

  bool CheckEmergencyDynamicSideAgentBaseRisk();

  bool is_dynamic_agent_emergency_avoidance_situation_ = false;
  RequestType recommend_dynamic_agent_emergency_avoidance_direction_ = NO_CHANGE;
  int dynamic_agent_emergency_situation_timetstamp_ = 0;
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  RequestType lane_change_direction_ = NO_CHANGE;
  int right_lane_nums_ = 0;
  int left_lane_nums_ = 0;
  HysteresisDecision emergency_lane_change_avoid_speed_ysteresis_;
  RiskLevel risk_level_ = RiskLevel::NO_RISK;
};

}  // namespace planning