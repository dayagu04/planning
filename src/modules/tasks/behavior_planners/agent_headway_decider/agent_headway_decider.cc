#include "agent_headway_decider.h"

namespace planning {

AgentHeadwayDecider::AgentHeadwayDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "AgentHeadwayDecider";
}

bool AgentHeadwayDecider::Execute() { return true; }

}  // namespace planning
