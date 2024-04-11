#include "lateral_offset_decider.h"
#include "planning_context.h"
namespace planning {

LateralOffsetDecider::LateralOffsetDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralOffsetDeciderConfig>();
  lateral_offset_calculator_ = LateralOffsetCalculator(config_builder);
}

bool LateralOffsetDecider::Execute() {
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  double lat_offset = 0.0;
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &target_state = coarse_planning_info.target_state;
  if (target_state == ROAD_LC_LCHANGE || target_state == ROAD_LC_RCHANGE) {
    lateral_offset_calculator_.Reset();
  } else {
    avoid_obstacle_maintainer_.Process(session_);
    lateral_offset_calculator_.Process(
        session_, avoid_obstacle_maintainer_.avd_car_past(),
        avoid_obstacle_maintainer_.avd_sp_car_past(),
        avoid_obstacle_maintainer_.dist_rblane(),
        avoid_obstacle_maintainer_.flag_avd());
  }

  LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  lat_offset = lateral_offset_calculator_.lat_offset();
  lateral_offset_decider_output.is_valid =
      config_.is_valid_lateral_offset && fabs(lat_offset) > 1e-2;
  lateral_offset_decider_output.lateral_offset = lat_offset;
  return true;
}

}  // namespace planning