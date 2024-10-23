#pragma once

#include "tasks/task.h"

namespace planning {

class LongitudinalDecisionDecider : public Task {
 public:
  LongitudinalDecisionDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  ~LongitudinalDecisionDecider() override = default;

  bool Execute() override;

 private:
  SccLonBehaviorPlannerConfig config_;
};

}  // namespace planning
