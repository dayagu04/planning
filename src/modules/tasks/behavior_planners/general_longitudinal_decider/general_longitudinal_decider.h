/**
 * @file longitudinal_decider.h
 **/

#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/trajectory1d/bounded_constant_jerk_trajectory1d.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"

namespace planning {

struct LonYieldInfo {
  double min_ds;
  double min_ttc;
  double min_acc;
  double min_jerk;
  bool keep_stop;
};

struct VelocityLimitInfo {
  double v_limit_map;
  double v_limit_usr;
  double v_limit_curv;
  double v_limit_narrow_area;
  double v_limit_final;
};

class GeneralLongitudinalDecider : public Task {
 public:
  explicit GeneralLongitudinalDecider(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~GeneralLongitudinalDecider() = default;

  bool Execute(planning::framework::Frame *frame) override;

 private:
  void get_lon_decision_info(LonDecisionInfo &lon_decision_information);

  BoundedConstantJerkTrajectory1d get_velocity_limit(
      const LonRefPath &lon_ref_path);

  const double compute_max_lat_acceleration() const;

  void set_velocity_acceleration_bound(LonRefPath &lon_ref_path);

  // TBD(binwang33): refpath_points 入参没有被使用，需要去除
  // 函数名需要统一
  void construct_longitudinal_obstacle_decisions(
      const TrajectoryPoints &traj_points,
      const ReferencePathPoints &refpath_points,
      ObstacleDecisions &obstacle_decisions, LonRefPath &lon_ref_path);

  // TBD(binwang33): refpath_points 入参没有被使用，需要去除
  // 函数名需要统一
  void construct_longitudinal_obstacle_decision(
      const TrajectoryPoints &traj_points,
      const ReferencePathPoints &refpath_points,
      const std::vector<planning_math::Polygon2d> &overlap_path,
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision, LonRefPath &lon_ref_path);

  void make_longitudinal_overlap_path(
      const TrajectoryPoints &traj_points,
      std::vector<planning_math::Polygon2d> &overlap_path);

  bool check_longitudinal_ignore_obstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);

  bool check_obstacle_both_sides(
      const std::shared_ptr<FrenetObstacle> obstacle);

  // TBD(binwang33): refpath_points 入参没有被使用，需要去除
  // 函数名需要统一
  void construct_refpath_points(const TrajectoryPoints &traj_points,
                                ReferencePathPoints &refpath_points);

  void construct_lon_decision_trajectory(
      const TrajectoryPoints &traj_points,
      TrajectoryPoints &lon_decision_traj_points);

  // outer decision
  void construct_longitudinal_outer_decision(
      const TrajectoryPoints &traj_points,
      const ReferencePathPoints &refpath_points,
      const CoarsePlanningInfo &coarse_planning_info,
      ObstacleDecisions &obstacle_decisions);

  static void prepare_obstacle_decision(ObstacleDecisions &obstacle_decisions,
                                        int obstacle_id);
  static void prepare_obstacle_position_decision(
      ObstacleDecision &obstacle_decision, const TrajectoryPoint &traj_pt);

  void generate_lon_decision_from_path(
      const TrajectoryPoints &lateral_trajectory_points,
      const ReferencePathPoints &refpath_points,
      ObstacleDecisions &obstacle_decisions, LonRefPath &lon_ref_path);

  double get_distance_to_destination();

  double get_narrow_area_velocity_limit();

  double get_s_bound_by_target_parking_space();

  /**
   * @brief 将纵向输出转存为proto后存入planning debug info
   */
  void GenerateLonRefPathPB(const LonRefPath &lon_ref_path);

 private:
  LongitudinalDeciderV3Config config_;
  AdaptiveCruiseControlConfig config_acc_;  // 暂时不考虑acc
  StartStopEnableConfig config_start_stop_;
  LonYieldInfo lon_yield_info_;
  VelocityLimitInfo vel_limit_info_;
  // CollisionChecker lon_collision_checker_; // 主要给pnp使用，暂时不需要
  planning::framework::Frame *frame_;
};

}  // namespace planning
