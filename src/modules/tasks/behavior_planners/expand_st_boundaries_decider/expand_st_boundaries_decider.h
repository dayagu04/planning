#pragma once

#include "common/speed/st_graph_input.h"
#include "tasks/task.h"

namespace planning {

class ExpandStBoundariesDecider : public Task {
 public:
  ExpandStBoundariesDecider(const EgoPlanningConfigBuilder *config_builder,
                            framework::Session *session);
  virtual ~ExpandStBoundariesDecider() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
