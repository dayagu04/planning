/**
 * @file longitudinal_motion_planner.h
 **/

#ifndef __SCC_LONGITUDINAL_MOTION_PLANNER_V3_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNER_V3_H__

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
#include "src/scc_longitudinal_motion_planning_cost_v3.h"
#include "src/scc_longitudinal_motion_planning_model_v3.h"
#include "src/scc_longitudinal_motion_planning_problem_v3.h"
#include "task_basic_types.h"
#include "tasks/task.h"

namespace planning {

class SccLongitudinalMotionPlannerV3 : public Task {
 public:
  SccLongitudinalMotionPlannerV3(const EgoPlanningConfigBuilder* config_builder,
                                 framework::Session* session);

  void Init();
  bool Execute() override;

 private:
  void AssembleInput();
  void Update();

  Bound s_limit_;
  SccLonMotionPlannerConfig config_;

  AdaptiveCruiseControlConfig config_acc_;
  StartStopEnableConfig config_start_stop_;

  std::string name_;
  std::shared_ptr<
      pnc::scc_longitudinal_planning_v3::SccLongitudinalMotionPlanningProblemV3>
      planning_problem_ptr_;
  planning::common::LongitudinalPlanningInput planning_input_;
};

}  // namespace planning

#endif
