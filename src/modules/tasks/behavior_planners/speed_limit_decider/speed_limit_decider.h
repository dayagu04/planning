#pragma once

#include "tasks/task.h"

namespace planning {

class SpeedLimitDecider : public Task {
 public:
  SpeedLimitDecider(const EgoPlanningConfigBuilder *config_builder,
                    framework::Session *session);
  virtual ~SpeedLimitDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
