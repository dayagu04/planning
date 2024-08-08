#include "longtime_task_pipeline_v1.h"
#include <memory>

namespace planning {

LongTimeTaskPipelineV1::LongTimeTaskPipelineV1(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  lateral_offset_decider_ =
      std::make_unique<LateralOffsetDecider>(config_builder, session);
  gap_selector_decider_ =
      std::make_unique<GapSelectorDecider>(config_builder, session);
  general_lateral_decider_ =
      std::make_unique<GeneralLateralDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<LateralMotionPlanner>(config_builder, session);
  agent_longitudinal_decider_ =
      std::make_unique<AgentLongitudinalDecider>(config_builder, session);
  scc_lon_behavior_planner_ =
      std::make_unique<SccLonBehaviorPlanner>(config_builder, session);
  scc_longitudinal_motion_planner_ =
      std::make_unique<SccLongitudinalMotionPlanner>(config_builder, session);
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
}

bool LongTimeTaskPipelineV1::Run() {
  bool ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }

  ok = lateral_offset_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_offset_decider_->Name());
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

  ok = agent_longitudinal_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }

  ok = scc_lon_behavior_planner_->Execute();
  if (!ok) {
    AddErrorInfo(scc_lon_behavior_planner_->Name());
    return false;
  }

  ok = scc_longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(scc_longitudinal_motion_planner_->Name());
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