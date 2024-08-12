/**
 * @file longtime_task_pipeline_v1.h
 * @brief This pipeline relies on localization module
 *        but does not rely on prediction module
 **/

#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "ego_planning_config.h"
#include "session.h"
#include "tasks/behavior_planners/agent_longitudinal_decider/agent_longitudinal_decider.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider.h"
#include "tasks/behavior_planners/scc_lon_behavior_planner/scc_lon_behavior_planner.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/scc_lon_motion_planner/scc_longitudinal_motion_planner.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"

namespace planning {

class LongTimeTaskPipelineV1 : public BaseTaskPipeline {
 public:
  explicit LongTimeTaskPipelineV1(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~LongTimeTaskPipelineV1() = default;

  bool Run() override;

 private:
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<LateralOffsetDecider> lateral_offset_decider_;
  std::unique_ptr<GapSelectorDecider> gap_selector_decider_;
  std::unique_ptr<GeneralLateralDecider> general_lateral_decider_;
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<SccLonBehaviorPlanner> scc_lon_behavior_planner_;
  std::unique_ptr<SccLongitudinalMotionPlanner>
      scc_longitudinal_motion_planner_;
  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;
  std::unique_ptr<AgentLongitudinalDecider> agent_longitudinal_decider_;
};

}  // namespace planning