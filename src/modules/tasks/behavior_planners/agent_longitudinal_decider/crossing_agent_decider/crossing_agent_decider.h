#pragma once

#include <memory>

#include "session.h"
#include "tasks/task.h"

namespace planning {

class CrossingAgentDecider : public Task {
 public:
  explicit CrossingAgentDecider(const EgoPlanningConfigBuilder* config_builder,
                                framework::Session* session);
  virtual ~CrossingAgentDecider() = default;

  bool Execute() override;

  bool Update();

  bool Reset();
};
}  // namespace planning