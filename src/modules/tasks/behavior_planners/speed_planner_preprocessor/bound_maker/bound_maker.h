#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"

namespace planning {

class BoundMaker {
 public:
  BoundMaker(const SpeedPlannerConfig& speed_planning_config,
             framework::Session *session);
  ~BoundMaker() = default;

  common::Status Run();

 private:
};

}  // namespace planning
