/**
 * @file longtime_task_pipeline_v2.h
 * @brief This pipeline relies on localization & prediction module
 **/

#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "ego_planning_config.h"
#include "session.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/longitudinal_motion_planner/longitudinal_motion_planner.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"

namespace planning {

class LongTimeTaskPipelineV2 : public BaseTaskPipeline {
 public:
  explicit LongTimeTaskPipelineV2(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~LongTimeTaskPipelineV2() = default;

  bool Run() override;

 private:
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<GapSelectorDecider> gap_selector_decider_;
  std::unique_ptr<GeneralLateralDecider> general_lateral_decider_;
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<GeneralLongitudinalDecider> general_longitudinal_decider_;
  std::unique_ptr<LongitudinalMotionPlanner> longitudinal_motion_planner_;
  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;
};

}  // namespace planning