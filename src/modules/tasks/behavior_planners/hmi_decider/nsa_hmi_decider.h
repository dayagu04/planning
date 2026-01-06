#pragma once

#include <vector>

#include "debug_info_log.h"
#include "hmi_decider.h"
#include "narrow_space_hmi/narrow_space_hmi_decider.h"
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class NSAHMIDecider : public HMIDecider {
 public:
  explicit NSAHMIDecider(const EgoPlanningConfigBuilder* config_builder,
                         framework::Session* session);

  virtual ~NSAHMIDecider() = default;

  bool Execute() override;

 private:
  std::shared_ptr<NarrowSpaceHMIDecider> narrow_space_hmi_decider_ = nullptr;
};
}  // namespace planning
