#pragma once
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class BaseGeneralLateralDecider : public Task {
 public:
  BaseGeneralLateralDecider(const EgoPlanningConfigBuilder *config_builder,
                             framework::Session *session);
  virtual ~BaseGeneralLateralDecider() = default;

  virtual bool Execute();

 protected:
  HppGeneralLateralDeciderConfig config_;
  planning::framework::Session *session_;
};
}  // namespace planning