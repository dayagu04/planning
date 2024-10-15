#pragma once

#include "tasks/task.h"

namespace planning {

class ClosestInPathVehicleDecider : public Task {
 public:
  ClosestInPathVehicleDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  virtual ~ClosestInPathVehicleDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
