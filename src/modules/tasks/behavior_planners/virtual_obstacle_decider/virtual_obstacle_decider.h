#pragma once

#include "tasks/task.h"

namespace planning {

class VirtualObstacleDecider : public Task {
 public:
  VirtualObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                         framework::Session *session);
  virtual ~VirtualObstacleDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
