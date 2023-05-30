/**
 * @file general_lateral_motion_planner.h
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
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_model.h"
#include "lateral_motion_planning_problem.h"
#include "math_lib.h"
#include "scenario/scenario_state_machine.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"
namespace planning {

class LateralMotionPlanner : public Task {
public:
  explicit LateralMotionPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~LateralMotionPlanner() = default;

  void Init();
  bool Execute(planning::framework::Frame *frame) override;

private:
  void GeneratePlanningInput();
  void GeneratePlanningOutput();
  void UpdateOneStepTrajectory();
  GeneralLateralMotionPlannerConfig config_;
  string name_;
  std::shared_ptr<pnc::lateral_planning::LateralMotionPlanningProblem>
      planning_problem_ptr_;
  planning::common::LateralPlanningOutput planning_output_;
  planning::common::LateralPlanningInput planning_input_;
  State init_state_;

  // state vector
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<double> theta_vec_;
  std::vector<double> delta_vec_;
  std::vector<double> omega_vec_;
  std::vector<double> curv_vec;
  std::vector<double> d_curv_vec;
  std::vector<double> s_vec_;
  double v_cruise_ = 0.0;
};

} // namespace planning
