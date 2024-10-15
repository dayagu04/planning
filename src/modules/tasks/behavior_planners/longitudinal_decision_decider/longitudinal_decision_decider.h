#pragma once

#include "tasks/task.h"

namespace planning {

class LongitudinalDecisionDecider : public Task {
 public:
  LongitudinalDecisionDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  virtual ~LongitudinalDecisionDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
