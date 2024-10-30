#include "parking_switch_decider.h"
#include <cmath>
#include "virtual_lane_manager.h"
#include "environmental_model.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {
ParkingSwitchDecider::ParkingSwitchDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "ParkingSwitchDecider";
}

bool ParkingSwitchDecider::Execute() {
  Clear();

  if (!session_->is_hpp_scene()) {
    return true;
  }

  const EnvironmentalModel &env = session_->environmental_model();
  const double distance_to_target_slot =
      env.get_route_info()->get_route_info_output().distance_to_target_slot;

  parking_switch_info_.dist_to_memory_slot = distance_to_target_slot;

  return true;
}

void ParkingSwitchDecider::Clear() {
  parking_switch_info_.Clear();
  return;
}

}  // namespace planning
