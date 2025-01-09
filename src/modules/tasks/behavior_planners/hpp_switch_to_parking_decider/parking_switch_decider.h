#pragma once

#include "hpp_switch_info.h"
#include "tasks/task.h"

namespace planning {

// stage1: If ego is stopping, try to switch parking.
// stage2: If ego is near memory slot and ego is moving, try to switch parking.
// stage3: Searching any slot in memory route.
class ParkingSwitchDecider : public Task {
 public:
  ParkingSwitchDecider(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session);
  virtual ~ParkingSwitchDecider() = default;

  bool Execute() override;

 private:
  HppParkingSwitchConfig config_;
  HppParkingSwitchInfo parking_switch_info_;
};

}  // namespace planning
