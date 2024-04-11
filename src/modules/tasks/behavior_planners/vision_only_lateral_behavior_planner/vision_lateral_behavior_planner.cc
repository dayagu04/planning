#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "planning_context.h"
namespace planning {

VisionLateralBehaviorPlanner::VisionLateralBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<VisionLateralBehaviorPlannerConfig>();
  name_ = "VisionLateralBehaviorPlanner";
}

bool VisionLateralBehaviorPlanner::Execute() {
  LOG_DEBUG("=======VisionLateralBehaviorPlanner======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto current_time = IflyTime::Now_ms();

  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();

  lane_width_ = 3.8;

  bool success = Process();

  if (!success) {
    LOG_DEBUG("VisionLateralBehaviorPlanner::execute failed");
  }
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("VisionLateralBehaviorPlanner", end_time - current_time);
  return success;
}

bool VisionLateralBehaviorPlanner::Process() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  update_avoid_cars(coarse_planning_info);
  return true;
}

}  // namespace planning