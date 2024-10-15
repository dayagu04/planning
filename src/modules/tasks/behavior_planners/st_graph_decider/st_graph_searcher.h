#pragma once

#include "tasks/task.h"

namespace planning {

class StGraphSearcher : public Task {
 public:
  StGraphSearcher(const EgoPlanningConfigBuilder *config_builder,
                  framework::Session *session);
  virtual ~StGraphSearcher() = default;

  bool Execute() override;

 private:
};

}  // namespace planning
