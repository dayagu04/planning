#include "realtime_task_pipeline_v1.h"

namespace planning {

RealtimeSccTaskPipeline::RealtimeSccTaskPipeline(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  vision_lateral_behavior_planner_ =
      std::make_unique<VisionLateralBehaviorPlanner>(config_builder, session);
  vision_lateral_motion_planner_ =
      std::make_unique<VisionLateralMotionPlanner>(config_builder, session);
  vision_longitudinal_behavior_planner_ =
      std::make_unique<VisionLongitudinalBehaviorPlanner>(config_builder,
                                                          session);
}

bool RealtimeSccTaskPipeline::Run() {
  bool ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }

  ok = vision_lateral_behavior_planner_->Execute();
  if (!ok) {
    AddErrorInfo(vision_lateral_behavior_planner_->Name());
    return false;
  }

  ok = vision_lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(vision_lateral_motion_planner_->Name());
    return false;
  }

  ok = vision_longitudinal_behavior_planner_->Execute();
  if (!ok) {
    AddErrorInfo(vision_longitudinal_behavior_planner_->Name());
    return false;
  }

  return true;
}

}  // namespace planning