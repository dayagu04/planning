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
#include "ilqr_core.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_model.h"
#include "lateral_motion_planning_problem.h"
#include "math_lib.h"
#include "scenario/scenario_state_machine.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"
namespace planning {

class GeneralLateralMotionPlanner : public Task {
 public:
  explicit GeneralLateralMotionPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~GeneralLateralMotionPlanner() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool generate_lat_motion_planner_input();

  bool generate_lat_motion_planner_output();

  bool update_one_step_traj();

  bool init();

 private:
  GeneralLateralMotionPlannerConfig config_;
  string name_;
  // std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  std::shared_ptr<pnc::lateral_planning::LateralMotionPlanningProblem>
      lat_motion_planning_problem_ptr_;
  planning::common::LateralMotionPlanningOutput planning_output_;
  planning::common::LateralMotionPlanningInput planning_input_;
  State init_state_;
};

}  // namespace planning
