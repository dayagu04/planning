#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"

namespace planning {

class TargetMaker {
 public:
  TargetMaker(const EgoPlanningConfigBuilder *config_builder,
              framework::Session *session);
  ~TargetMaker() = default;

  common::Status Run();



  

 private:
};

}  // namespace planning
