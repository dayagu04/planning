#pragma once

#include <vector>

#include "debug_info_log.h"
#include "hmi_decider.h"
#include "lateral_avoid_hmi/lateral_avoid_hmi_decider.h"
#include "obstacle_brake_hmi/obstacle_brake_hmi_decider.h"
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class RADSHMIDecider : public HMIDecider {
 public:
  explicit RADSHMIDecider(const EgoPlanningConfigBuilder* config_builder,
                          framework::Session* session);

  virtual ~RADSHMIDecider() = default;

  bool Execute() override;

 private:
  std::shared_ptr<LateralAvoidHMIDecider> lateral_avoid_hmi_decider_ = nullptr;
  std::shared_ptr<ObstacleBrakeHMIDecider> obstacle_brake_hmi_decider_ = nullptr;
};
}  // namespace planning