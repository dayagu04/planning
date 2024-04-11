#include "longtime_task_pipeline_v2.h"

namespace planning {

LongTimeTaskPipelineV2::LongTimeTaskPipelineV2(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  gap_selector_decider_ =
      std::make_unique<GapSelectorDecider>(config_builder, session);
  general_lateral_decider_ =
      std::make_unique<GeneralLateralDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<LateralMotionPlanner>(config_builder, session);
  general_longitudinal_decider_ =
      std::make_unique<GeneralLongitudinalDecider>(config_builder, session);
  longitudinal_motion_planner_ =
      std::make_unique<LongitudinalMotionPlanner>(config_builder, session);
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
}

bool LongTimeTaskPipelineV2::Run() {
  bool ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }

  ok = gap_selector_decider_->Execute();
  if (!ok) {
    AddErrorInfo(gap_selector_decider_->Name());
    return false;
  }

  ok = general_lateral_decider_->Execute();
  if (!ok) {
    AddErrorInfo(general_lateral_decider_->Name());
    return false;
  }

  ok = lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }

  ok = general_longitudinal_decider_->Execute();
  if (!ok) {
    AddErrorInfo(general_longitudinal_decider_->Name());
    return false;
  }

  ok = longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(longitudinal_motion_planner_->Name());
    return false;
  }

  ok = result_trajectory_generator_->Execute();
  if (!ok) {
    AddErrorInfo(result_trajectory_generator_->Name());
    return false;
  }

  return true;
}

}  // namespace planning