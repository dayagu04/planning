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

using namespace planning_math;

class GeneralLateralDecider : public Task {
 public:
  explicit GeneralLateralDecider(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);

  virtual ~GeneralLateralDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();
  void UnitTest();

 private:
  bool CalCruiseVelByCurvature(const double ego_v,
                               const CoarsePlanningInfo& coars_planning_info,
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
  double CalculateExtraDecreaseBuffer(
      const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left);
  double CalculateExtraLaneTypeDecreaseBuffer(bool is_nudge_left,
                                              const double start_s,
                                              const double end_s);
  void CalculateExtraLaneWidthDecreaseBuffer();

  bool CheckObstacleNudgeDecision(
      const std::shared_ptr<FrenetObstacle> &obstacle);

  bool CheckObstacleCrossingCondition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void RefineConflictLatDecisions(const double &ego_l,
                                  ObstacleDecision &obstacle_decision);
  bool HackYawSideObstacle(const std::shared_ptr<FrenetObstacle> obstacle,
                           bool is_nudge_left, double &limit_overlap_min_y,
                           double &limit_overlap_max_y);
  bool IsCutoutSideObstacle(const std::shared_ptr<FrenetObstacle> obstacle,
                            double &limit_overlap_min_y,
                            double &limit_overlap_max_y);
  double CalLaneWidth();
  iflyauto::LaneBoundaryType CalLaneBoundaryType(const LineDirection direction,
                                                 const double s) const;
  void PostProcessReferenceTrajBySoftBound(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);
  void ExtractBoundary(
      std::vector<std::pair<double, double>> &frenet_soft_bounds,
      std::vector<std::pair<double, double>> &frenet_hard_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info);
  void ProtectBoundByInitPoint(std::pair<double, double> &bound,
                               std::pair<BoundInfo, BoundInfo> &bound_info);
  void ExtractDynamicObstacleBound(const ObstacleDecision &obstacle_decision);
  void ExtractStaticObstacleBound(const ObstacleDecision &obstacle_decision);

  void PostProcessBound(const double planning_init_point_l,
                        const std::vector<WeightedBound> &bounds_input,
                        std::pair<double, double> &bound_output,
                        std::pair<BoundInfo, BoundInfo> &bound_info);
  void SaveLatDebugInfo(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info,
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void GenerateObstaclePreliminaryDecision(
      double ego_l, double distance_to_right_lane_border,
      double distance_to_left_lane_border, double overlap_min_y,
      double overlap_max_y, double lat_buf_dis, bool b_overlap_side,
      bool init_lon_no_overlap, bool is_nudge_left, bool is_cross_obj,
      LatObstacleDecisionType pre_lateral_decision,
      bool &reset_conflict_decision, ObstacleDecision &obstacle_decision,
      LatObstacleDecisionType &lat_decision,
      LonObstacleDecisionType &lon_decision);
  void AddObstacleDecisionBound(int id, double t, BoundType bound_type,
                                double overlap_min_y, double overlap_max_y,
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
  void HandleAvoidScene(TrajectoryPoints &traj_points,
                        double dynamic_ref_buffer);
  void CalcLateralBehaviorOutput();
  bool IsLonOverlap(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFarObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsRearObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsBlockedObstacleInLaneBorrow(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForStaticObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForDynamicObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsAgentPredLonOverlapWithPlanPath(
      const std::shared_ptr<FrenetObstacle> obstacle);
  double CalculateSideObstacleExtraDecreaseBufferInIntersection(
      const std::shared_ptr<FrenetObstacle> obstacle,
      bool is_nudge_left, bool in_intersection);
  double AdjustBufferForSideObstacleInIntersection(
    const std::shared_ptr<FrenetObstacle> obstacle,
    double overlap_min_y, double overlap_max_y,
    double lat_buf_dis, bool is_nudge_left,
    double rear_lon_buf_dis, double front_lon_buf_dis,
    LatObstacleDecisionType lat_decision, int index);
  void CalculateAvoidObstacles(
    const std::vector<std::pair<double, double>> frenet_soft_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info);
  double CalStaticNudgeLatBufDis(
    const std::shared_ptr<FrenetObstacle> obstacle, bool in_intersection,
    bool is_nudge_left, double overlap_min_y, double overlap_max_y,
    bool is_side_obstacle, double extra_lane_type_decrease_buffer,
    bool is_update_hard_bound, double ego_width, double lane_width);

 private:
  GeneralLateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  TrajectoryPoints ref_traj_points_;
  TrajectoryPoints plan_history_traj_;
  std::unordered_map<int, std::vector<int>> match_index_map_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> last_lat_obstacle_decision_;

  ReferencePathPoints ref_path_points_;
  ObstacleDecisions static_obstacle_decisions_;
  ObstacleDecisions dynamic_obstacle_decisions_;

  std::vector<WeightedBounds> soft_bounds_;
  std::vector<WeightedBounds> hard_bounds_;

  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;
  double min_road_radius_ = 10000.0;
  double cruise_vel_ = 0.0;
  double extra_lane_width_decrease_buffer_ = 0.0;
  double overlap_start_s_ = 0.0;
  double overlap_end_s_ = 0.0;
  bool is_lane_change_scene_ = false;
  bool is_blocked_obstacle_ = false;
  LatDeciderLaneChangeInfo lat_lane_change_info_ =
      LatDeciderLaneChangeInfo::NONE;
  planning::common::LateralBehaviorDebugInfo lat_debug_info_;
};

}  // namespace planning
