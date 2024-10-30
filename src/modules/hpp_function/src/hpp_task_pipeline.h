#pragma once

#include <memory>

// #include "apa_function/apa_plan_interface.h"
#include "base_task_pipeline.h"
#include "behavior_planners/hpp_switch_to_parking_decider/parking_switch_decider.h"
#include "ego_planning_config.h"
#include "session.h"
#include "tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "tasks/behavior_planners/hpp_general_lateral_decider/hpp_general_lateral_decider.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/lateral_obstacle_decider.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/longitudinal_motion_planner/longitudinal_motion_planner.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"

namespace planning {

class HppTaskPipeline : public BaseTaskPipeline {
 public:
  HppTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                  framework::Session *session);

  virtual ~HppTaskPipeline() = default;

  bool Run() override;

 private:
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<LateralObstacleDecider> lateral_obstacle_decider_;
  std::unique_ptr<HppGeneralLateralDecider> hpp_general_lateral_decider_;
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<GeneralLongitudinalDecider> general_longitudinal_decider;
  std::unique_ptr<LongitudinalMotionPlanner> longitudinal_motion_planner_;
  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;
  std::unique_ptr<ParkingSwitchDecider> parking_switch_decider_;
};

}  // namespace planning