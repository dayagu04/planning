#pragma once

#include <math.h>
#include "scenario/ego_planning_candidate.h"

#include "context/ego_planning_config.h"
#include "session.h"

namespace planning {

class StartStopEnable {
 public:
  StartStopEnable(const EgoPlanningConfigBuilder *config_builder,
                  framework::Session *session) {
    session_ = session;
    config_ = config_builder->cast<StartStopEnableConfig>();
    init();
  }

  virtual ~StartStopEnable() = default;

  void go_trajectory(LonDecisionInfo &lon_decision_information,
                     int &start_stop_information,
                     StartStopInfo &start_stop_result);
  bool enable_start_stop() {
    return enable_hnp_functions_ && enable_start_stop_function_;
  }

 private:
  void init();
  bool enable_start_stop_function_ = false;
  bool enable_hnp_functions_ = false;
  StartStopEnableConfig config_;
  framework::Session *session_ = nullptr;
};

}  // namespace planning