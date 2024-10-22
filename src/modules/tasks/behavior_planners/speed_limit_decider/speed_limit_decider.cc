#include "speed_limit_decider.h"

namespace planning {
SpeedLimitDecider::SpeedLimitDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "SpeedLimitDecider";
}
bool SpeedLimitDecider::Execute() { return true; }
}  // namespace planning
