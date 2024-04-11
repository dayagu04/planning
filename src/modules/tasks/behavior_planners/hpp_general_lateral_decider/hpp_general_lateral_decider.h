/**
 * @file lateral_decider.h
 **/

#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "history_obstacle_manager.h"
#include "lateral_behavior_planner.pb.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "quintic_poly_path.h"
// #include "scenario_state_machine.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "tasks/task_interface/general_lateral_decider_output.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
namespace planning {

class HppGeneralLateralDecider : public Task {
 public:
  explicit HppGeneralLateralDecider(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~HppGeneralLateralDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();

 private:
  // bool process(ObstacleDecisions &obstacle_decisions,
  //              TrajectoryPoints &traj_points);

  // // 1. construct the trajectory of reference and bind the obstacle info on
  bool ConstructReferencePathPoints(const TrajectoryPoints &traj_points);

  // // 2. construct the obstacle decisions
  void ConstructLateralObstacleDecisions(
      // const TrajectoryPoints &traj_points,
      ObstacleDecisions &obstacle_decisions);

  void ConstructLateralObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision);
  // 3. construct the lane and boundary bound
  void ConstructLaneAndBoundaryBounds(
      MapObstacleDecision &map_obstacle_decisions);

  bool CheckObstacleNudgeCondition(
      const std::shared_ptr<FrenetObstacle> &obstacle);

  bool CheckObstacleCrossingCondition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void RefineConflictLatDecisions(const double &ego_l,
                                  ObstacleDecision &obstacle_decision);

  void ExtractBoundary(
      const MapObstacleDecision &map_obstacle_decision,
      const ObstacleDecisions &obstacle_decisions,
      std::vector<std::pair<double, double>> &frenet_safe_bounds,
      std::vector<std::pair<double, double>> &frenet_path_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void GenerateEnuBoundaryPoints(
      const std::vector<std::pair<double, double>> &frenet_safe_bounds,
      const std::vector<std::pair<double, double>> &frenet_path_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void SampleRoadDistanceInfo(const double &s_target,
                              ReferencePathPoint &sample_path_point);

  void GenerateEnuReferenceTraj(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void GenerateEnuReferenceTheta(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void HandleLaneChangeScene(TrajectoryPoints &traj_points);

  void CalcLateralBehaviorOutput();

  void ConstructStaticObstacleTotalPolygons(
      std::vector<std::pair<int, planning_math::Polygon2d>>
          &left_groundline_polygons,
      std::vector<std::pair<int, planning_math::Polygon2d>>
          &right_groundline_polygons,
      std::vector<std::pair<int, planning_math::Polygon2d>>
          &left_parking_space_polygons,
      std::vector<std::pair<int, planning_math::Polygon2d>>
          &right_parking_space_polygons);

  ObstacleBorderInfo GetNearestObstacleBorder(
      const planning_math::Polygon2d &care_polygon, double care_area_s_start,
      double care_area_s_end,
      const std::vector<std::pair<int, planning_math::Polygon2d>>
          &obstacle_frenet_polygons,
      bool is_left, bool is_sorted, bool is_curve, int index,
      const TrajectoryPoints &traj_points);

  HppGeneralLateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  TrajectoryPoints ref_traj_points_;
  ReferencePathPoints ref_path_points_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> cur_reference_path_ptr_;
  std::shared_ptr<HistoryObstacleManager> history_obstacle_manager_;
  double cruise_vel_ = 0.0;
  bool is_lane_change_scene_ = false;
  bool is_deduce_near_obstacles_ = true;
  LatDeciderLaneChangeInfo lat_lane_change_info_ =
      LatDeciderLaneChangeInfo::NONE;
  planning::common::LateralBehaviorDebugInfo lat_debug_info_;
};

}  // namespace planning
