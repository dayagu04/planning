/**
 * @file general_general_lateral_motion_planner.h
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
#include "ilqr_core.h"
#include "longitudinal_motion_planner.pb.h"
#include "longitudinal_motion_planning_cost.h"
#include "longitudinal_motion_planning_model.h"
#include "longitudinal_motion_planning_problem.h"
#include "math_lib.h"
#include "scc_function/mrc_condition.h"
#include "scenario/scenario_state_machine.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"

namespace planning {

class LongitudinalMotionPlanner : public Task {
public:
  explicit LongitudinalMotionPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~LongitudinalMotionPlanner() = default;

  bool Execute(planning::framework::Frame *frame) override;
  void Init();
  void GeneratePlanningInput();
  void GeneratePlanningOutput();

  // bool RecheckPlanningResult();

  void interpolate_frenet_lon(const std::vector<TrajectoryPoint> &traj_points,
                              const std::vector<double> &s,
                              std::vector<double> &l,
                              std::vector<double> &heading_angle,
                              std::vector<double> &curvature);

  bool UpdateOneStepTrajectory();

  bool init();

private:
  Bound s_limit_;
  ILqrLonMotionPlannerConfig config_;
  AdaptiveCruiseControlConfig config_acc_;
  StartStopEnableConfig config_start_stop_;
  string name_;
  std::shared_ptr<pnc::longitudinal_planning::LongitudinalMotionPlanningProblem>
      planning_problem_ptr_;
  LongitudinalMotionPlanning::PlanningOutput planning_output_;
  LongitudinalMotionPlanning::PlanningInput planning_input_;
  State init_state_;

  // state vector
  std::vector<double> s_vec_;
  std::vector<double> v_vec_;
  std::vector<double> a_vec_;
  std::vector<double> j_vec_;
  std::vector<double> t_vec_;
};

} // namespace planning
