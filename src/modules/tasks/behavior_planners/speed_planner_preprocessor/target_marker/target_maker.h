#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "target.h"

namespace planning {

class TargetMaker {
 public:
  TargetMaker(const EgoPlanningConfigBuilder *config_builder,
              framework::Session *session);
  ~TargetMaker() = default;

  bool Run();

 private:
  SpeedPlannerConfig config_;
  framework::Session *session_;
};

}  // namespace planning
