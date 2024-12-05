#include "agent_headway_decider_output.h"

namespace planning {

void AgentHeadwayDeciderOutput::Reset() {
  // agents_headway_map_.clear();
  return;
}

const std::unordered_map<int32_t, AgentHeadwayInfo>&
AgentHeadwayDeciderOutput::agents_headway_Info() const {
  return agents_headway_map_;
}

void AgentHeadwayDeciderOutput::set_agents_headway_Info(
    const std::unordered_map<int32_t, AgentHeadwayInfo>& agents_headway_map) {
  agents_headway_map_ = agents_headway_map;
}

}  // namespace planning
