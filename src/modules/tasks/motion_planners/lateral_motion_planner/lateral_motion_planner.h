/**
 * @file lateral_motion_planner.h
 **/

#ifndef __LATERAL_MOTION_PLANNER_H__
#define __LATERAL_MOTION_PLANNER_H__

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
// #include "scenario_state_machine.h"
#include "src/lateral_motion_planning_cost.h"
#include "src/lateral_motion_planning_model.h"
#include "src/lateral_motion_planning_problem.h"
#include "src/lateral_motion_planning_weight.h"
#include "task_basic_types.h"
#include "tasks/task.h"
namespace planning {

class LateralMotionPlanner : public Task {
 public:
  LateralMotionPlanner(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session);

  void Init();
  bool Execute() override;

 private:
  void AssembleInput();
  void Update();

  LateralMotionPlannerConfig config_;
  string name_;
  std::shared_ptr<pnc::lateral_planning::LateralMotionPlanningProblem>
      planning_problem_ptr_;
  planning::common::LateralPlanningInput planning_input_;

  std::shared_ptr<pnc::lateral_planning::LateralMotionPlanningWeight>
      planning_weight_ptr_;
  double auto_start_time_;
};

}  // namespace planning

#endif
