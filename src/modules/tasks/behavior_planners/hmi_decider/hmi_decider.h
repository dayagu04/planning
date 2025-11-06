#pragma once

#include <vector>
#include "debug_info_log.h"
#include "planning_context.h"
#include "tasks/task.h"
#include "session.h"
#include "cone_warning_hmi/cone_warning_hmi.h"
#include "construction_warning_hmi/construction_warning_hmi.h"
namespace planning {

class HMIDecider : public Task {
 public:
  explicit HMIDecider(const EgoPlanningConfigBuilder *config_builder,
                               framework::Session *session);

  virtual ~HMIDecider() = default;

  bool Execute();
  
 private:
  std::shared_ptr<ConeWarningHMIDecider> cone_warning_hmi_decider_;
  std::shared_ptr<ConstructionWarningHMIDecider> construction_warning_hmi_decider_;
};
}  // namespace planning