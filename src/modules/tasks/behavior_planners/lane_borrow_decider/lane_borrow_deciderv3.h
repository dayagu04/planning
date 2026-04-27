#pragma once

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "agent/agent.h"
#include "common/utils/hysteresis_decision.h"
#include "config/vehicle_param.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "pose2d.h"
#include "reference_path.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tasks/behavior_planners/dp_path_decider/dp_road_graph.h"
#include "tasks/behavior_planners/lane_borrow_decider/lane_borrow_deciderv3_utils.h"
#include "tasks/task.h"
#include "virtual_lane.h"

namespace planning {
namespace lane_borrow_deciderV3 {

class LaneBorrowDecider : public Task {
 public:
  LaneBorrowDecider(const EgoPlanningConfigBuilder* config_builder,
                    framework::Session* session)
      : Task(config_builder, session) {
    config_ = config_builder->cast<LaneBorrowDeciderConfig>();
    path_decider_ = std::make_unique<DPRoadGraph>(config_builder, session);
    rule_path_decider_ = std::make_unique<
        planning::lane_borrow_deciderv3_utils::LaneBorrowDeciderV3Utils>(
        config_builder, session);
  };
  virtual ~LaneBorrowDecider() = default;

  bool Execute() override;

 private:
  bool ProcessEnvInfos();
  void LaneTypeDistanceInfo();
  void Update();
  void LogDebugInfo();
  bool RunPathPlanning(bool is_need_stabilize = true);
  bool RunPathPlanningBaseDP();
  bool RunPathPlanningBaseRules(bool is_need_stabilize = true);
  bool CheckIfNoBorrowToLaneBorrowDriving();
  bool CheckIfLaneBorrowToNoBorrow();
  bool CheckIfLaneBorrowToLaneBorrowCrossing();
  bool CheckIfLaneBorrowCrossingToNoBorrow();
  bool IsSafeForBackOriginLane();
  bool CheckIfBackOriginLaneToNoBorrow();
  bool CheckIfBackOriginLaneToLaneBorrowDriving();
  bool CheckIfBackOriginLaneToLaneBorrowCrossing();

  bool CheckLaneBorrowCondition();
  void UpdateJunctionInfo();
  bool IsLaneTypeDashedOrMixed(const iflyauto::LaneBoundaryType& type);
  bool UpdateLaneBorrowDirection();
  bool SelectStaticBlockingObstcales();
  bool ObstacleDecision();
  bool IfChangeTargetLane();
  void CheckKeyObstaclesIntention(const agent::Agent* agent, bool& is_cut_in,
                                  bool& is_cut_out);
  bool CheckLeadObs();
  bool UpdateDynamicBlockingObstacles();
  BorrowDirection GetBypassDirection(
      const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id);
  BorrowDirection GetPredBypassDirection(
      const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id,
      const double delta_t);
  bool CheckLaneBorrowDircetion();
  bool EnoughSafetyDistance();
  bool CrossingPositionJudgment();

  void ClearLaneBorrowStatus();
  bool CheckBackWardObs();
  bool IsNeedResetObserve(LaneBorrowFailedReason reason);
  void CheckBlockingObstaclesIntention(int32 obs_id, bool& is_borrow);
  Box2d PredictBoxPosition(const agent::Agent* agent, double delta_t);
  FrenetObstacleBoundary GetSLboundaryFromAgent(const Box2d& obs_box);
  bool CheckVirtualLaneSuppressBorrow();
  void SendObserveToLatFlag();
  void SendHMIData();
  bool CheckBlockedBorrowObstaclesByTrajectory();
  void UpdateBorrowDirectionMap();
  bool CheckObserveTime();
  void UpdateObstacleLateralDecisionBaseDPPath();
  void UpdateObstacleLateralDecisionBaseRulePath();
  bool CheckLaneBorrowByTrajectory(
      const std::shared_ptr<FrenetObstacle>& lead_nearest_follow_obstacle);
  bool CheckSpatioTemporalPlanner();
  bool IsRearObsSafeByArrivalTime(int32_t rear_obs_id,
                                  const FrenetObstacleBoundary& rear_obs_sl,
                                  double rear_obs_v);

  LaneBorrowStatus lane_borrow_status_{kNoLaneBorrow};
  double forward_solid_start_dis_{1000.0};
  double forward_solid_end_s_{1500.0};

  double distance_to_stop_line_{1000.0};
  double distance_to_cross_walk_{1000.0};
  double dis_to_traffic_lights_{10000.0};
  double obs_left_l_{-10.0};
  double obs_right_l_{10.0};
  double obs_start_s_{10.0};
  double obs_end_s_{-10.0};

  bool left_borrow_{false};
  bool right_borrow_{false};
  bool is_first_frame_to_lane_borrow_{false};
  bool is_facility_{false};
  BorrowDirection bypass_direction_{NO_BORROW};
  BorrowDirection path_direction_{NO_BORROW};
  planning::common::IntersectionState intersection_state_ =
      planning::common::NO_INTERSECTION;

  std::pair<double, double> ego_sl_;  // s, l
  FrenetEgoState ego_sl_state_;
  double ego_speed_;
  int64_t front_id_{0};

  FrenetBoundary ego_frenet_boundary_;
  LaneBorrowDeciderOutput lane_borrow_decider_output_;
  double heading_angle_{0.0};
  Pose2D ego_xy_;

  int observe_frame_num_{0};
  std::unordered_map<int, int> obs_observe_frame_map_;
  int observe_path_frame_num_{0};
  std::unordered_map<int, std::pair<BorrowDirection, int>> obs_direction_map_;
  std::unordered_map<int32_t, int32_t> lat_flag_map_;
  // Hysteresis cache for dp-path-based lateral decisions to avoid frame jitter.
  // pair<stable_decision, consecutive_switch_count>
  std::unordered_map<int32_t, std::pair<LatObstacleDecisionType, int>>
      dp_lat_decision_hysteresis_map_{};
  int lane_change_state_{0};

  std::shared_ptr<FrenetObstacle> nearest_no_borrow_obstacle_{nullptr};
  std::vector<int> static_blocked_obj_id_vec_;  // after decision
  std::vector<int> last_static_blocked_obj_id_vec_;
  std::vector<std::shared_ptr<FrenetObstacle>> static_blocked_obstacles_;
  std::shared_ptr<ReferencePath> current_reference_path_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> current_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ptr_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ptr_ = nullptr;
  LaneBorrowDeciderConfig config_;
  std::unique_ptr<DPRoadGraph> path_decider_;
  std::unique_ptr<
      planning::lane_borrow_deciderv3_utils::LaneBorrowDeciderV3Utils>
      rule_path_decider_;
  iflyauto::LaneBoundaryType left_lane_boundary_type_;
  iflyauto::LaneBoundaryType right_lane_boundary_type_;
  // hmi cnt
  int start_frame_{0};
  bool nudging_prompt_{false};
  bool takeover_prompt_{false};
  int spatio_temporal_planner_intersection_count_ = 0;
  int virtual_area_count_ = 0;
  HysteresisDecision lane_borrow_hmi_speed_hysteresis_{0.5, 0.3};
  HysteresisDecision lane_borrow_hmi_boundary_dist_hysteresis_{-0.6, -0.8};
  bool is_hold_reset_path_ = false;
  LaneBorrowFailedReason last_lane_borrow_failed_reason_ = NONE_FAILED_REASON;
};
}  // namespace lane_borrow_deciderV3
}  // namespace planning