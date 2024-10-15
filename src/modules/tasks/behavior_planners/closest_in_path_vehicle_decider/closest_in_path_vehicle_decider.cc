#include "closest_in_path_vehicle_decider.h"

namespace planning {
ClosestInPathVehicleDecider::ClosestInPathVehicleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "ClosestInPathVehicleDecider";
}

bool ClosestInPathVehicleDecider::Execute() { return true; }
}  // namespace planning
