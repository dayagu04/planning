#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"

namespace planning {

class BoundMaker {
 public:
  BoundMaker(const EgoPlanningConfigBuilder *config_builder,
             framework::Session *session);
  ~BoundMaker() = default;

  common::Status Run();

 private:
};

}  // namespace planning
