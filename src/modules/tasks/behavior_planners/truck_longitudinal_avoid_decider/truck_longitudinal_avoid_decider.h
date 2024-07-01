#pragma once

#include "tasks/task.h"

namespace planning {

class TruckLongitudinalAvoidDecider : public Task {
 public:
  TruckLongitudinalAvoidDecider(const EgoPlanningConfigBuilder *config_builder,
                                framework::Session *session);
  virtual ~TruckLongitudinalAvoidDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
