/**
 * @file lateral_decider.h
 **/

#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/math/linear_interpolation.h"
#include "context/environmental_model.h"
#include "context/frenet_ego_state.h"
#include "context/obstacle_manager.h"
#include "scenario/scenario_state_machine.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"

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
  explicit GeneralLateralDecider(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~GeneralLateralDecider() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool ExecuteTest(planning::framework::Frame *frame, bool pipeline_test);

  bool init_info();

 private:
  // bool process(ObstacleDecisions &obstacle_decisions,
  //              TrajectoryPoints &traj_points);

  // // 1. construct the trajectory of reference and bind the obstacle info on
  // the
  // // traj point
  bool construct_reference_path_points(ReferencePathPoints &refpath_points);

  // // 2. construct the obstacle decisions
  void construct_lateral_obstacle_decisions(
      // const TrajectoryPoints &traj_points,
      const ReferencePathPoints &refpath_points,
      ObstacleDecisions &obstacle_decisions);

  void construct_lateral_obstacle_decision(
      const ReferencePathPoints &refpath_points,
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision);
  // 3. construct the lane and boundary bound
  void construct_lane_and_boundary_bounds(
      const ReferencePathPoints &refpath_points,
      MapObstacleDecision &map_obstacle_decisions);

  // void construct_lat_behavior_output(
  //     const ObstacleDecisions
  //         &obstacle_decisions);  // output the info for lat motion planner

  bool check_obj_nudge_condition(
      const ReferencePathPoints &refpath_points,
      const std::shared_ptr<FrenetObstacle> &obstacle);

  bool check_obj_crossing_condition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void refine_conflict_lat_decisions(const double &ego_l,
                                     ObstacleDecision &obstacle_decision);

  void generate_boundary(const MapObstacleDecision &map_obstacle_decision,
                         const ObstacleDecisions &obstacle_decisions,
                         const ReferencePathPoints &refpath_points,
                         LatDeciderOutput &lat_decider_output);

  void generate_lat_reference_traj(const ReferencePathPoints &refpath_points,
                                   LatDeciderOutput &lat_decider_output);

  void handle_lane_change_scene();

  LateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  planning::framework::Frame *frame_;
  LatDeciderInfo lat_decider_info_;
  LatDeciderOutput lat_decider_output_;
  std::vector<std::shared_ptr<FrenetObstacle>> obs_vec_;
  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> cur_reference_path_ptr_;
  double cruise_vel_;
  double horizion_num_;
  double delta_t_;
  bool is_lane_change_scene_;
};

}  // namespace planning
