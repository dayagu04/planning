#include "truck_longitudinal_avoid_decider.h"

namespace planning {
TruckLongitudinalAvoidDecider::TruckLongitudinalAvoidDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "TruckLongitudinalAvoidDecider";
}

bool TruckLongitudinalAvoidDecider::Execute() { return true; }
}  // namespace planning
