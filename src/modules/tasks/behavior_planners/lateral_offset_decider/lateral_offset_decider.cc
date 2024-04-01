#include "lateral_offset_decider.h"
#include "planning_context.h"
namespace planning {

LateralOffsetDecider::LateralOffsetDecider(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
  lateral_offset_calculator_ = LateralOffsetCalculator(config_builder);
}

bool LateralOffsetDecider::Execute(planning::framework::Frame *frame) {
  if (Task::Execute(frame) == false) {
    return false;
  }
  double lat_offset = 0.0;
  const auto &coarse_planning_info = pipeline_context_->coarse_planning_info;
  const auto &target_state = coarse_planning_info.target_state;
  if (target_state == ROAD_LC_LCHANGE || target_state == ROAD_LC_RCHANGE) {
    lateral_offset_calculator_.Reset();
  } else {
    avoid_obstacle_maintainer_.Process(frame, pipeline_context_);
    lateral_offset_calculator_.Process(
        frame, pipeline_context_, avoid_obstacle_maintainer_.avd_car_past(),
        avoid_obstacle_maintainer_.avd_sp_car_past(),
        avoid_obstacle_maintainer_.dist_rblane(),
        avoid_obstacle_maintainer_.flag_avd());
  }

  LateralOffsetDeciderOutput &lateral_offset_decider_output =
      frame_->mutable_session()
          ->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  lat_offset = lateral_offset_calculator_.lat_offset();
  lateral_offset_decider_output.is_valid =
      config_.is_valid_lateral_offset && fabs(lat_offset) > 1e-2;
  lateral_offset_decider_output.lateral_offset = lat_offset;
  return true;
}

}  // namespace planning