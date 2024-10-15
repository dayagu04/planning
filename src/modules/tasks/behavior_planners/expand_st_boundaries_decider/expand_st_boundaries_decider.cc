#include "expand_st_boundaries_decider.h"

namespace planning {

ExpandStBoundariesDecider::ExpandStBoundariesDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "ExpandStBoundariesDecider";
}

bool ExpandStBoundariesDecider::Execute() { return true; }
}  // namespace planning
