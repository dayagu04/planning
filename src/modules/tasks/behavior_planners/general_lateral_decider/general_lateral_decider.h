/**
 * @file lateral_decider.h
 **/

#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "quintic_poly_path.h"
#include "scenario_state_machine.h"
#include "spline_projection.h"
#include "task.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
namespace planning {

class GeneralLateralDecider : public Task {
 public:
  explicit GeneralLateralDecider(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~GeneralLateralDecider() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool ExecuteTest(planning::framework::Frame *frame, bool pipeline_test);

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
      std::vector<std::pair<double, double>> &frenet_path_bounds);

  void GenerateEnuBoundaryPoints(

      const std::vector<std::pair<double, double>> &frenet_safe_bounds,
      const std::vector<std::pair<double, double>> &frenet_path_bounds,
      LatDeciderOutput &lat_decider_output);

  void SampleRoadDistanceInfo(const double &s_target,
                              ReferencePathPoint &sample_path_point);

  void GenerateEnuReferenceTraj(LatDeciderOutput &lat_decider_output);

  void GenerateEnuReferenceTheta(LatDeciderOutput &lat_decider_output);

  void HandleLaneChangeScene(TrajectoryPoints &traj_points);

  void CalcLateralBehaviorOutput();

  GeneralLateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  planning::framework::Frame *frame_;
  TrajectoryPoints ref_traj_points_;
  ReferencePathPoints ref_path_points_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> cur_reference_path_ptr_;
  double cruise_vel_ = 0.0;
  bool is_lane_change_scene_ = false;
  LatDeciderLaneChangeInfo lat_lane_change_info_ =
      LatDeciderLaneChangeInfo::NONE;
};

}  // namespace planning
