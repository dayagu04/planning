/**
 * @file longitudinal_motion_planner.h
 **/

#ifndef __LONGITUDINAL_MOTION_PLANNER_H__
#define __LONGITUDINAL_MOTION_PLANNER_H__

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "adas_function/mrc_condition.h"
#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "ilqr_core.h"
#include "longitudinal_motion_planner.pb.h"
#include "math_lib.h"
#include "obstacle_manager.h"
// #include "scenario_state_machine.h"
#include "src/longitudinal_motion_planning_cost.h"
#include "src/longitudinal_motion_planning_model.h"
#include "src/longitudinal_motion_planning_problem.h"
#include "task_basic_types.h"
#include "tasks/task.h"

namespace planning {

class LongitudinalMotionPlanner : public Task {
 public:
  LongitudinalMotionPlanner(const EgoPlanningConfigBuilder *config_builder,
                            framework::Session *session);

  void Init();
  bool Execute() override;

 private:
  void AssembleInput();
  void Update();

  Bound s_limit_;
  LongitudinalMotionPlannerConfig config_;

  AdaptiveCruiseControlConfig config_acc_;
  StartStopEnableConfig config_start_stop_;

  string name_;
  std::shared_ptr<pnc::longitudinal_planning::LongitudinalMotionPlanningProblem>
      planning_problem_ptr_;
  planning::common::LongitudinalPlanningInput planning_input_;
};

}  // namespace planning

#endif
