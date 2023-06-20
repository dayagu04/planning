/**
 * @file lateral_decider.h
 **/

#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "math/quintic_poly_2d.h"
#include "obstacle_manager.h"
#include "task.h"
#include "task_basic_types.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {

// struct LatYieldInfo {
//   double min_ds;
//   double min_ttc;
//   double min_acc;
//   double min_jerk;
//   bool keep_stop;
// };

struct LatDeciderInfo {
  double desired_vel;
  double l_care_width;
  double care_obj_lat_distance_threshold;
  double care_obj_lon_distance_threshold;
  double dynamic_obj_safe_buffer;
  double vehicle_width_with_rearview_mirror;
  double vehicle_length;
  double rear_bumper_to_rear_axle;
  double min_gain_vel;
  double min_obstacle_avoid_distance;
  double lateral_bound_converge_speed;
  double kPhysicalBoundWeight;
  double kHardBoundWeight;
  double dynamic_bound_slack_coefficient;
  double l_offset_limit;
  double buffer2lane;
  double buffer2border;
};

// struct VelocityLimitInfo {
//   double v_limit_map;
//   double v_limit_usr;
//   double v_limit_curv;
//   double v_limit_narrow_area;
//   double v_limit_final;
// };

class GeneralLateralDecider : public Task {
 public:
  explicit GeneralLateralDecider(const EgoPlanningConfigBuilder *config_builder,
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

  void ConstructLateralObstacleDecision(const std::shared_ptr<FrenetObstacle> obstacle,
                                        ObstacleDecision &obstacle_decision);
  // 3. construct the lane and boundary bound
  void ConstructlaneAndBoundaryBounds(MapObstacleDecision &map_obstacle_decisions);

  bool CheckObstacleNudgeCondition(const std::shared_ptr<FrenetObstacle> &obstacle);

  bool CheckObstacleCrossingCondition(const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void RefineConflictLatDecisions(const double &ego_l, ObstacleDecision &obstacle_decision);

  void ExtractBoundary(const MapObstacleDecision &map_obstacle_decision, const ObstacleDecisions &obstacle_decisions,
                       std::vector<std::pair<double, double>> &frenet_safe_bounds,
                       std::vector<std::pair<double, double>> &frenet_path_bounds);

  void GenerateEnuBoundaryPoints(

      const std::vector<std::pair<double, double>> &frenet_safe_bounds,
      const std::vector<std::pair<double, double>> &frenet_path_bounds, LatDeciderOutput &lat_decider_output);

  void sample_road_distance_info(const double &s_target, double &left_lane_distance, double &right_lane_distance,
                                 double &left_road_distance, double &right_road_distance);

  void GenerateEnuReferenceTraj(LatDeciderOutput &lat_decider_output);

  void GenerateEnuReferenceTheta(LatDeciderOutput &lat_decider_output);

  void HandleLaneChangeScene();

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
};

}  // namespace planning
