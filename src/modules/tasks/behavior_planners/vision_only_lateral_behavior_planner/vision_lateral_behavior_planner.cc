#include "src/modules/tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"

namespace planning {

VisionLateralBehaviorPlanner::VisionLateralBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<VisionLateralBehaviorPlannerConfig>();
  name_ = "VisionLateralBehaviorPlanner";
}

bool VisionLateralBehaviorPlanner::Execute(planning::framework::Frame *frame) {
  frame_ = frame;

  if (Task::Execute(frame) == false) {
    return false;
  }

  // auto &ego_prediction_result = pipeline_context_->planning_result;
  auto &ego_planning_info = pipeline_context_->planning_info;
  auto &coarse_planning_info = pipeline_context_->coarse_planning_info;

  auto &virtual_lane_manager = frame_->mutable_session()
                                   ->mutable_environmental_model()
                                   ->get_virtual_lane_manager();

  // TODO:update width()
  // lane_width_ =
  // virtual_lane_manager->get_lane_with_virtual_id(coarse_planning_info.target_lane_id)->width();
  lane_width_ = 3.8;

  bool success =
      Process(coarse_planning_info, ego_planning_info.lateral_avd_cars_info);

  if (!success) {
    LOG_DEBUG("VisionLateralBehaviorPlanner::execute failed");
  }
  return success;
}

bool VisionLateralBehaviorPlanner::Process(
    const CoarsePlanningInfo &coarse_planning_info,
    LateralAvdCarsInfo &lateral_avd_cars_info) {
  update_avoid_cars(coarse_planning_info, lateral_avd_cars_info);
  return true;
}

}  // namespace planning