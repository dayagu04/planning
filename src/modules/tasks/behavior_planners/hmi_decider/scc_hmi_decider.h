#pragma once

#include <vector>

#include "cone_warning_hmi/cone_warning_hmi.h"
#include "construction_warning_hmi/construction_warning_hmi.h"
#include "debug_info_log.h"
#include "hmi_decider.h"
#include "lane_change_hmi/lane_change_hmi_decider.h"
#include "longitudinal_hmi/longitudinal_hmi_decider.h"
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

class SCCHMIDecider : public HMIDecider {
 public:
  explicit SCCHMIDecider(const EgoPlanningConfigBuilder* config_builder,
                         framework::Session* session);

  virtual ~SCCHMIDecider() = default;

  bool Execute() override;

 private:
  std::shared_ptr<ConeWarningHMIDecider> cone_warning_hmi_decider_;
  std::shared_ptr<ConstructionWarningHMIDecider>
      construction_warning_hmi_decider_;
  std::shared_ptr<LaneChangeHmiDecider> lane_change_hmi_decider_ = nullptr;
  std::shared_ptr<LongitudinalHmiDecider> longitudinal_hmi_decider_ = nullptr;
};
}  // namespace planning