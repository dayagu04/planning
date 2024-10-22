#include "longitudinal_decision_decider.h"

namespace planning {

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongitudinalDecisionDecider";
}

bool LongitudinalDecisionDecider::Execute() { return true; }

}  // namespace planning
