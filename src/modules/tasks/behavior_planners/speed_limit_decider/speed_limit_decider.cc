#include "speed_limit_decider.h"

namespace planning {
SpeedLimitDecider::SpeedLimitDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "SpeedLimitDecider";
}
bool SpeedLimitDecider::Execute() {
  LOG_DEBUG("=======SpeedLimitDecider======= \n");
  // 1. speed limit from map: ramp & split
  CalculateMapSpeedLimit();
  // 2. speed limit from curvature/ lateral_acceleration
  CalculateCurveSpeedLimit();
  // 3. speed limit from static agents
  CalculateStaticAgentLimit();
  // 4. speed limit from intersection
  CalculateIntersectionSpeedLimit();
  // 5. speed limit from uncertainty of perception: perception visibility
  CalculatePerceptVisibSpeedLimit();
  // 6. speed limit from POI
  CalculatePOISpeedLimit();
  return true;
}

void SpeedLimitDecider::CalculateCurveSpeedLimit() {}

void SpeedLimitDecider::CalculateMapSpeedLimit() {}

void SpeedLimitDecider::CalculateStaticAgentLimit() {}

void SpeedLimitDecider::CalculateIntersectionSpeedLimit() {}

void SpeedLimitDecider::CalculatePerceptVisibSpeedLimit() {}

void SpeedLimitDecider::CalculatePOISpeedLimit() {}

}  // namespace planning
