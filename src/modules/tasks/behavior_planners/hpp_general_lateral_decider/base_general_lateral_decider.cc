#include "base_general_lateral_decider.h"

namespace planning {
BaseGeneralLateralDecider::BaseGeneralLateralDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  session_ = session;
}

bool BaseGeneralLateralDecider::Execute() { return true; }
}  // namespace planning