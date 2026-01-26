#pragma once

#include <vector>

#include "debug_info_log.h"
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class HMIDecider : public Task {
 public:
  explicit HMIDecider(const EgoPlanningConfigBuilder* config_builder,
                      framework::Session* session);

  virtual ~HMIDecider() = default;

  virtual bool Execute();

 protected:
  HmiDeciderConfig config_;
};
}  // namespace planning