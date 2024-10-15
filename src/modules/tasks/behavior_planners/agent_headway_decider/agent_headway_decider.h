#pragma once

#include "tasks/task.h"

namespace planning {

class AgentHeadwayDecider : public Task {
 public:
  AgentHeadwayDecider(const EgoPlanningConfigBuilder *config_builder,
                      framework::Session *session);
  virtual ~AgentHeadwayDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
