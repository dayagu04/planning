/**
 * @file realtime_lateral_motion_planner.h
 **/

#ifndef __REALTIME_LATERAL_MOTION_PLANNER_H__
#define __REALTIME_LATERAL_MOTION_PLANNER_H__

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_motion_planner.pb.h"
#include "math_lib.h"
#include "obstacle_manager.h"
#include "scenario_state_machine.h"
#include "src/realtime_lateral_motion_planning_cost.h"
#include "src/realtime_lateral_motion_planning_model.h"
#include "src/realtime_lateral_motion_planning_problem.h"
#include "task.h"
#include "task_basic_types.h"
namespace planning {

class RealtimeLateralMotionPlanner : public Task {
 public:
  RealtimeLateralMotionPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  void Init();
  bool Execute(planning::framework::Frame *frame);

 private:
  void AssembleInput();
  void Update();

  RealtimeLateralMotionPlannerConfig config_;
  string name_;
  std::shared_ptr<
      pnc::realtime_lateral_planning::RealtimeLateralMotionPlanningProblem>
      planning_problem_ptr_;
  planning::common::LateralPlanningInput planning_input_;
};

}  // namespace planning

#endif
