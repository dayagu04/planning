#include "crossing_agent_decider.h"

namespace planning {

CrossingAgentDecider::CrossingAgentDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "CrossingAgentDecider";
}

bool CrossingAgentDecider::Execute() {
  LOG_DEBUG("=======CrossingAgentDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  // Update
  Update();

  return true;
}

bool CrossingAgentDecider::Update() { return true; }
}  // namespace planning