/**
 * @file lateral_decider.h
 **/

#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_behavior_planner.pb.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "quintic_poly_path.h"
// #include "scenario_state_machine.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
namespace planning {

class GeneralLateralDecider : public Task {
 public:
  explicit GeneralLateralDecider(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);

  virtual ~GeneralLateralDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();

 private:
  bool CalCruiseVelByCurvature(const double ego_v,
                               const std::vector<double> &d_poly,
                               double &cruise_v);

  void ConstructTrajPoints(TrajectoryPoints &traj_points);

  // // 1. construct the trajectory of reference and bind the obstacle info on
  bool ConstructReferencePathPoints(const TrajectoryPoints &traj_points);

  // 2. construct the lane and boundary bound
  void GenerateRoadAndLaneBoundary();
  void UpdateDistanceToRoadBorder();
  void GenerateRoadHardSoftBoundary();
  void GenerateLaneSoftBoundary();

  void GetDesireRoadExtraBuffer(double *const left_road_extra_buffer,
                                double *const right_road_extra_buffer);
  void GetLateralTTCToRoad(double *max_collision_t,
                           double *const left_collision_t,
                           double *const right_collision_t);

  // 3. construct the obstacle decisions
  void GenerateObstaclesBoundary();
  void GenerateStaticObstaclesBoundary(
      const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
      ObstacleDecisions &obstacle_decisions);
  void GenerateStaticObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision, bool is_update_hard_bound);
  void GenerateDynamicObstaclesBoundary(
      const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
      ObstacleDecisions &obstacle_decisions);
  void GenerateDynamicObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision);

  bool CheckObstacleNudgeDecision(
      const std::shared_ptr<FrenetObstacle> &obstacle);

  bool CheckObstacleCrossingCondition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void RefineConflictLatDecisions(const double &ego_l,
                                  ObstacleDecision &obstacle_decision);

  void ExtractBoundary(
      std::vector<std::pair<double, double>> &frenet_soft_bounds,
      std::vector<std::pair<double, double>> &frenet_hard_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info);
  void ProtectBoundByInitPoint(std::pair<double, double> &bound,
                               std::pair<BoundInfo, BoundInfo> &bound_info);
  void ExtractDynamicObstacleBound(const ObstacleDecision &obstacle_decision);
  void ExtractStaticObstacleBound(const ObstacleDecision &obstacle_decision);

  void PostProcessBound(std::vector<WeightedBound> &bounds_input,
                        std::pair<double, double> &bound_output,
                        std::pair<BoundInfo, BoundInfo> &bound_info);
  void SaveLatDebugInfo(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info);

  void GenerateObstaclePreliminaryDecision(
      double ego_l, double distance_to_right_lane_border,
      double distance_to_left_lane_border, double overlap_min_y,
      double overlap_max_y, double lat_buf_dis, bool b_overlap_side,
      bool init_lon_no_overlap, bool is_nudge_left, bool is_cross_obj,
      LatObstacleDecisionType pre_lateral_decision,
      bool &reset_conflict_decision, ObstacleDecision &obstacle_decision,
      LatObstacleDecisionType &lat_decision,
      LonObstacleDecisionType &lon_decision);
  void AddObstacleDecisionBound(int id, double t,
                                Polygon2d &care_overlap_polygon,
                                double lat_buf_dis,
                                LatObstacleDecisionType lat_decision,
                                LonObstacleDecisionType lon_decision,
                                ObstacleDecision &obstacle_decision,
                                bool is_update_hard_bound = false);
  void GenerateLateralDeciderOutput(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);
  void GenerateEnuBoundaryPoints(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void SampleRoadDistanceInfo(const double &s_target,
                              ReferencePathPoint &sample_path_point);

  void GenerateEnuReferenceTraj(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void GenerateEnuReferenceTheta(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void HandleLaneChangeScene(TrajectoryPoints &traj_points);
  void HandleAvoidScene(TrajectoryPoints &traj_points);
  void CalcLateralBehaviorOutput();
  bool IsFarObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsRearObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForStaticObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForDynamicObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsAgentPredLonOverlapWithPlanPath(
      const std::shared_ptr<FrenetObstacle> obstacle);

 private:
  GeneralLateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  TrajectoryPoints ref_traj_points_;
  TrajectoryPoints plan_history_traj_;
  std::unordered_map<int, std::vector<int>> match_index_map_;

  ReferencePathPoints ref_path_points_;
  ObstacleDecisions static_obstacle_decisions_;
  ObstacleDecisions dynamic_obstacle_decisions_;

  std::vector<WeightedBounds> soft_bounds_;
  std::vector<WeightedBounds> hard_bounds_;

  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;
  double cruise_vel_ = 0.0;
  bool is_lane_change_scene_ = false;
  LatDeciderLaneChangeInfo lat_lane_change_info_ =
      LatDeciderLaneChangeInfo::NONE;
  planning::common::LateralBehaviorDebugInfo lat_debug_info_;
};

}  // namespace planning
