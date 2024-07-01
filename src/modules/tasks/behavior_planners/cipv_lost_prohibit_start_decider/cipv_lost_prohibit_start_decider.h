#pragma once

#include "tasks/task.h"

namespace planning {

class CipvLostProhibitStartDecider : public Task {
 public:
  CipvLostProhibitStartDecider(const EgoPlanningConfigBuilder *config_builder,
                               framework::Session *session);
  virtual ~CipvLostProhibitStartDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
