#pragma once

#include <list>
#include <unordered_set>
#include "agent.h"
#include "ego_planning_config.h"
#include "session.h"

namespace planning {
namespace agent {

class AgentManager {
 public:
  AgentManager() = default;
  AgentManager(const EgoPlanningConfigBuilder* config_builder,
               planning::framework::Session* session);

  ~AgentManager() = default;

  void SetConfig(const EgoPlanningConfigBuilder* config_builder);

  void Update(const double start_timestamp_s);

  // The 'Update' method will clear and update the 'current_agents_'
  //   void Update(const double start_timestamp_s,
  //               const std::unordered_map<int32_t, Agent>& agent_table);

  // The 'Append' method will just append the agent in 'current_agents_'
  void Append(const std::unordered_map<int32_t, Agent>& agent_table);

  void Reset();

  const std::vector<const Agent*>& GetAllCurrentAgents() const;

  Agent* mutable_agent(const int32_t id);

  const Agent* GetAgent(const int32_t id) const;

  const Agent* GetHistoryAgentByIndex(const int32_t id,
                                      const int32_t prev_index) const;

  const std::unordered_set<int32_t>& GetAgentSet() const;

 private:
  void DeleteOlderAgent();

 private:
  planning::framework::Session* session_ = nullptr;
  EgoPlanningObstacleManagerConfig config_;
  std::vector<const Agent*> current_agents_;
  std::unordered_set<int32_t> current_agents_ids_;
  std::unordered_map<int32_t, std::list<std::unique_ptr<Agent>>>
      historical_agents_;
};

}  // namespace agent
}  // namespace planning