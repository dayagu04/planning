#pragma once

#include "agent/agent.h"
#include "session.h"

namespace planning {

class AgentTrajectoryCalculator {
 public:
  AgentTrajectoryCalculator(framework::Session* session);

  ~AgentTrajectoryCalculator() = default;

  bool Process();

 private:
  bool GeneratePurePursuitTrajectory(agent::Agent* ptr_agent);

 private:
  framework::Session* session_ = nullptr;
};

}  // namespace planning
