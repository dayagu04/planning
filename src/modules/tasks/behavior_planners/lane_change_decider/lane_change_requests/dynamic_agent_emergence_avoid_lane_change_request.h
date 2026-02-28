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

  // 0.5g刹停检测紧急情况更新函数
  void UpdateBrakeFailureEmergencySituation();

  void GenerateLaneChangeDirection();

  void CheckLaneChangeDirection(int lc_status);

  bool CheckEmergencyBaseLastEmergencyAvoid();

  bool CheckEmergencyDynamicSideAgentBaseRisk();

  // 步骤1：检测0.5g减速度无法刹停的障碍物
  bool CheckEmergencyBrakeFailureObstacle(int& obstacle_id);

  // 步骤2：检查目标车道安全性
  bool CheckTargetLaneSafety(RequestType direction, int target_lane_virtual_id);

  // 步骤3：确定变道方向（基于0.5g刹停检测）
  void GenerateLaneChangeDirectionByBrakeFailure(
      const bool is_left_lane_change_safe,
      const bool is_right_lane_change_safe);

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
  bool is_brake_failure_detected_ = false;  // 0.5g刹停失败标志
  int brake_failure_obstacle_id_ = -1;  // 刹停失败的障碍物ID
  int brake_failure_situation_timestamp_ = 0;  // 刹停失败情况时间戳（用于两帧滞回）
};

}  // namespace planning