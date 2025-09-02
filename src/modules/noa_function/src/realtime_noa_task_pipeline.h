#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "ego_planning_config.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "tasks/behavior_planners/vision_only_longitudinal_behavior_planner/vision_longitudinal_behavior_planner.h"

namespace planning {

class RealtimeNoaTaskPipeline : public BaseTaskPipeline {
 public:
  RealtimeNoaTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                          framework::Session *session);

  virtual ~RealtimeNoaTaskPipeline() = default;

  bool Run() override;

 private:
//   std::unique_ptr<LaneChangeDecider> lane_change_decider_;
//   std::unique_ptr<VisionLateralBehaviorPlanner>
//       vision_lateral_behavior_planner_;
//   std::unique_ptr<VisionLateralMotionPlanner> vision_lateral_motion_planner_;
//   std::unique_ptr<VisionLongitudinalBehaviorPlanner>
//       vision_longitudinal_behavior_planner_;
};

}  // namespace planning